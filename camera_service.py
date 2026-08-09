#!/usr/bin/env python3
"""
camera_service.py — robust always-on multi-camera server for HST.

Design goals (matching the robustness of the BNL setup):
  - Runs independently of the supervisor TUI. Start it once (systemd or
    tmux) and it stays up. Quitting the TUI never touches it.
  - Serves an MJPEG HTTP stream per camera directly (no separate viewer
    process, no AYLP-over-UDP hop). Anyone on the network can watch, and
    multiple viewers / SSH sessions all work at once.
  - Self-healing: if a camera errors or drops off USB, that camera's
    worker retries reconnection forever. One camera failing never affects
    the others.
  - Live exposure / gain: a small HTTP control endpoint applies changes
    inside the capture loop, where the ASI SDK reliably honours them.

HTTP endpoints (per camera, on its http_port):
    GET /                 HTML page with the <img> stream
    GET /stream           multipart MJPEG
    GET /stream?raw=1     raw pixel values (no MIN-MAX stretch)
    GET /stats            JSON: {min,max,mean,width,height,frames,exposure,gain}
    GET /set?exposure=US  set exposure in microseconds (also &gain=G)
    GET /set?gain=G       set gain

Configuration is the CAMERAS list below. Edit to match your hardware.

Usage:
    ZWO_ASI_LIB=/path/to/libASICamera2.so.1.41 python3 camera_service.py

Stop with Ctrl-C (or systemctl --user stop hst-cameras).
"""

import http.server
import json
import socket
import logging
import os
import signal
import socketserver
import struct
import sys
import threading
import time
import urllib.parse

import numpy as np

# ── configuration ─────────────────────────────────────────────────────────────
# match_name : SDK name string. Worker waits for this camera to appear.
# do_debayer : True for colour (MC), False for mono (MM).
# The final pixel count (after bin_hw, debayer, sw_bin) is not UDP-limited
# here because we serve MJPEG directly, but keep it reasonable for bandwidth.

CAMERAS = [
    dict(
        match_name  = 'ZWO ASI662MM',   # CONFOCAL — mono
        name        = 'CONFOCAL',
        http_port   = 9290,
        exposure_us = 5000,
        gain        = 0,
        bin_hw      = 2,
        roi_width   = 1920,
        roi_height  = 1080,
        roi_x       = 0,
        roi_y       = 0,
        sw_bin      = 2,
        do_debayer  = False,
    ),
    dict(
        match_name  = 'ZWO ASI662MC',   # FIBER — colour
        name        = 'FIBER',
        http_port   = 9663,
        exposure_us = 5000,
        gain        = 0,
        bin_hw      = 1,
        roi_width   = 1920,
        roi_height  = 1080,
        roi_x       = 0,
        roi_y       = 0,
        sw_bin      = 3,
        do_debayer  = True,
    ),
]

JPEG_QUALITY = 85

# The ASI SDK is not safe for concurrent calls across threads (opening
# cameras, enumerating, etc). Serialize ALL SDK calls behind one lock.
_SDK_LOCK = threading.Lock()

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(levelname)-8s  %(name)s: %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stderr,
)
log = logging.getLogger('cameras')


# ── image processing ──────────────────────────────────────────────────────────

def debayer_grey(img):
    """Average 2x2 Bayer quads → greyscale. img is HxW uint8. Returns H/2 x W/2."""
    a = img.astype(np.uint16)
    grey = (a[0::2, 0::2] + a[0::2, 1::2] + a[1::2, 0::2] + a[1::2, 1::2]) >> 2
    return grey.astype(np.uint8)


def bin_grey(img, factor):
    """Integer-average bin an HxW image by factor. uint32 accumulation only."""
    if factor <= 1:
        return img
    h, w = img.shape
    h2, w2 = h // factor, w // factor
    a = img[:h2 * factor, :w2 * factor].astype(np.uint32)
    a = a.reshape(h2, factor, w2, factor).sum(axis=(1, 3))
    a //= (factor * factor)
    return a.astype(np.uint8)


# ── per-camera worker ─────────────────────────────────────────────────────────

class CameraWorker:
    """Owns one camera. Self-healing capture loop + MJPEG HTTP server."""

    def __init__(self, cfg, asi):
        self.cfg  = cfg
        self.asi  = asi
        self.name = cfg['name']

        # live-tunable, protected by _lock
        self._lock        = threading.Lock()
        self._exposure_us = cfg['exposure_us']
        self._gain        = cfg['gain']
        self._pending_exp = cfg['exposure_us']
        self._pending_gn  = cfg['gain']

        # latest encoded frames + stats, protected by _flock
        self._flock       = threading.Lock()
        self._jpeg_norm   = None
        self._jpeg_raw    = None
        self._stats       = (0, 0, 0.0)
        self._shape       = (0, 0)
        self._frames      = 0
        self._ready       = threading.Event()

        self._stop        = threading.Event()
        self._fps_last_t  = time.time()
        self._fps_last_n  = 0

    # ── public control ────────────────────────────────────────────────────────

    def set_exposure(self, us):
        with self._lock:
            self._pending_exp = max(1, int(us))

    def set_gain(self, g):
        with self._lock:
            self._pending_gn = max(0, int(g))

    def stop(self):
        self._stop.set()

    # ── capture loop with reconnection ────────────────────────────────────────

    def run(self):
        # HTTP server for this camera runs continuously, independent of camera
        self._start_http()
        while not self._stop.is_set():
            try:
                self._capture_session()
            except Exception as exc:
                log.warning('%s: session ended (%s); retrying in 2s',
                            self.name, exc)
                time.sleep(2.0)

    def _find_index(self):
        """Return the ASI index whose name matches, or None."""
        with _SDK_LOCK:
            n = self.asi.get_num_cameras()
            for i in range(n):
                try:
                    cam = self.asi.Camera(i)
                    nm = cam.get_camera_property()['Name']
                    cam.close()
                    if nm == self.cfg['match_name']:
                        return i
                except Exception:
                    continue
            return None

    def _capture_session(self):
        idx = self._find_index()
        if idx is None:
            # camera not present yet — wait and retry (self-healing)
            time.sleep(2.0)
            return

        with _SDK_LOCK:
            cam = self.asi.Camera(idx)
            log.info('%s: opened index %d (%s)', self.name, idx,
                     cam.get_camera_property()['Name'])
            with self._lock:
                exp = self._pending_exp
                gn  = self._pending_gn
                self._exposure_us = exp
                self._gain        = gn

            cam.set_control_value(self.asi.ASI_EXPOSURE, exp, auto=False)
            cam.set_control_value(self.asi.ASI_GAIN, gn, auto=False)
            cam.set_control_value(self.asi.ASI_BANDWIDTHOVERLOAD, 40, auto=False)
            cam.set_roi(
                start_x=self.cfg['roi_x'] // self.cfg['bin_hw'],
                start_y=self.cfg['roi_y'] // self.cfg['bin_hw'],
                width=self.cfg['roi_width']  // self.cfg['bin_hw'],
                height=self.cfg['roi_height'] // self.cfg['bin_hw'],
                bins=self.cfg['bin_hw'],
                image_type=self.asi.ASI_IMG_RAW8,
            )
            cam.start_video_capture()
            log.info('%s: streaming — http://0.0.0.0:%d/',
                     self.name, self.cfg['http_port'])
        try:

            h0 = self.cfg['roi_height'] // self.cfg['bin_hw']
            w0 = self.cfg['roi_width']  // self.cfg['bin_hw']

            import cv2

            consecutive_timeouts = 0
            MAX_TIMEOUTS = 10   # ~20s of no frames (2s timeout each) -> reconnect

            while not self._stop.is_set():
                # apply any pending exposure/gain BETWEEN frames
                with self._lock:
                    need_exp = self._pending_exp != self._exposure_us
                    need_gn  = self._pending_gn  != self._gain
                    new_exp, new_gn = self._pending_exp, self._pending_gn
                if need_exp or need_gn:
                    with _SDK_LOCK:
                        if need_exp:
                            cam.set_control_value(
                                self.asi.ASI_EXPOSURE, new_exp, auto=False)
                        if need_gn:
                            cam.set_control_value(
                                self.asi.ASI_GAIN, new_gn, auto=False)
                    with self._lock:
                        if need_exp:
                            self._exposure_us = new_exp
                            log.info('%s exposure -> %d us', self.name, new_exp)
                        if need_gn:
                            self._gain = new_gn
                            log.info('%s gain -> %d', self.name, new_gn)

                try:
                    raw = cam.get_video_data(timeout=2000)
                except self.asi.ZWO_IOError:
                    consecutive_timeouts += 1
                    if consecutive_timeouts >= MAX_TIMEOUTS:
                        raise RuntimeError(
                            f'{self.name}: no frames for '
                            f'{consecutive_timeouts * 2}s, forcing reconnect')
                    continue
                except Exception:
                    # camera likely dropped — break to reconnect
                    raise

                consecutive_timeouts = 0
                img = np.frombuffer(raw, dtype=np.uint8).reshape(h0, w0)

                if self.cfg['do_debayer']:
                    img = debayer_grey(img)
                if self.cfg['sw_bin'] > 1:
                    img = bin_grey(img, self.cfg['sw_bin'])

                self._encode_and_store(img, cv2)

        finally:
            with _SDK_LOCK:
                try:
                    cam.stop_video_capture()
                except Exception:
                    pass
                try:
                    cam.close()
                except Exception:
                    pass

    def _encode_and_store(self, img, cv2):
        h, w = img.shape
        mn, mx, mean = int(img.min()), int(img.max()), float(img.mean())

        if mx > mn:
            norm = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        else:
            norm = img

        # overlay stats
        bgr = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
        with self._lock:
            exp, gn = self._exposure_us, self._gain
        cv2.putText(bgr, f'{self.name} {w}x{h}  exp{exp}us g{gn}',
                    (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(bgr, f'min{mn} max{mx} avg{mean:.0f}',
                    (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 0, 255) if mx >= 255 else (0, 255, 0), 1, cv2.LINE_AA)

        ok1, jn = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        ok2, jr = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok1:
            return
        with self._flock:
            self._jpeg_norm = jn.tobytes()
            self._jpeg_raw  = jr.tobytes() if ok2 else self._jpeg_norm
            self._stats     = (mn, mx, mean)
            self._shape     = (h, w)
            self._frames   += 1
        self._ready.set()

        # Log real FPS every ~5 seconds so we can see actual throughput
        now = time.time()
        if now - self._fps_last_t >= 5.0:
            fps = (self._frames - self._fps_last_n) / (now - self._fps_last_t)
            log.info('%s: %.1f fps  (frame %d)', self.name, fps, self._frames)
            self._fps_last_t = now
            self._fps_last_n = self._frames

    # ── HTTP server ───────────────────────────────────────────────────────────

    def _start_http(self):
        worker = self

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *a):
                pass

            def handle_one_request(self):
                try:
                    super().handle_one_request()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    self.close_connection = True

            def _send(self, code, ctype, body):
                try:
                    self.send_response(code)
                    self.send_header('Content-Type', ctype)
                    self.send_header('Content-Length', str(len(body)))
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

            def do_GET(self):
                p  = urllib.parse.urlparse(self.path)
                qs = urllib.parse.parse_qs(p.query)

                if p.path == '/':
                    body = (
                        b'<html><body style="margin:0;background:#000;text-align:center">'
                        b'<img src="/stream" style="max-width:100%;max-height:100vh;'
                        b'image-rendering:pixelated"></body></html>')
                    self._send(200, 'text/html', body)
                    return

                if p.path == '/set':
                    msg = []
                    if 'exposure' in qs:
                        worker.set_exposure(int(qs['exposure'][0]))
                        msg.append(f"exposure={qs['exposure'][0]}")
                    if 'gain' in qs:
                        worker.set_gain(int(qs['gain'][0]))
                        msg.append(f"gain={qs['gain'][0]}")
                    self._send(200, 'text/plain',
                               ('OK ' + ' '.join(msg)).encode())
                    return

                if p.path == '/stats':
                    with worker._flock:
                        mn, mx, mean = worker._stats
                        h, w = worker._shape
                        fr = worker._frames
                    with worker._lock:
                        exp, gn = worker._exposure_us, worker._gain
                    body = json.dumps(dict(
                        min=mn, max=mx, mean=round(mean, 1),
                        width=w, height=h, frames=fr,
                        exposure=exp, gain=gn)).encode()
                    self._send(200, 'application/json', body)
                    return

                if p.path == '/stream':
                    use_raw = qs.get('raw', ['0'])[0] == '1'
                    # Disable Nagle's algorithm: without this, MJPEG frames
                    # get batched by TCP and the stream looks like it runs
                    # in bursts (play a second, freeze, play again).
                    try:
                        self.connection.setsockopt(
                            socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    except Exception:
                        pass
                    self.send_response(200)
                    self.send_header(
                        'Content-Type',
                        'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    try:
                        while True:
                            if not worker._ready.wait(timeout=2.0):
                                continue
                            worker._ready.clear()
                            with worker._flock:
                                data = worker._jpeg_raw if use_raw else worker._jpeg_norm
                            if data is None:
                                continue
                            try:
                                self.wfile.write(
                                    b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n'
                                    b'Content-Length: ' + str(len(data)).encode()
                                    + b'\r\n\r\n' + data + b'\r\n')
                                self.wfile.flush()
                            except (BrokenPipeError, ConnectionResetError, OSError):
                                break
                    except Exception:
                        pass
                    return

                self._send(404, 'text/plain', b'not found')

        class ThreadingHTTP(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        srv = ThreadingHTTP(('0.0.0.0', self.cfg['http_port']), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True,
                         name=f'http-{self.name}').start()
        log.info('%s: HTTP server on :%d', self.name, self.cfg['http_port'])


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    sdk = os.environ.get(
        'ZWO_ASI_LIB',
        '/home/qshanty/git/aylp_asi/libasi/lib/x64/libASICamera2.so.1.41')

    import zwoasi as asi
    asi.init(sdk)
    log.info('SDK initialised: %s', sdk)
    log.info('Cameras present at startup: %d', asi.get_num_cameras())

    workers = [CameraWorker(cfg, asi) for cfg in CAMERAS]

    threads = []
    for w in workers:
        t = threading.Thread(target=w.run, daemon=True, name=f'worker-{w.name}')
        t.start()
        threads.append(t)

    def shutdown(sig, frm):
        log.info('Shutting down...')
        for w in workers:
            w.stop()
        time.sleep(0.5)
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    log.info('Camera service running. Endpoints:')
    for cfg in CAMERAS:
        log.info('  %-10s http://0.0.0.0:%d/', cfg['name'], cfg['http_port'])

    # keep alive
    while True:
        time.sleep(3600)


if __name__ == '__main__':
    main()
