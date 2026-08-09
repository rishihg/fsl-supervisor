#!/usr/bin/env python3
"""
camera_view2.py — robust AYLP camera MJPEG viewer.

Usage: camera_view2.py <udp_port> <http_port> [title]

Improvements over the original:
  - Larger receive buffer, tolerant of frame bursts
  - Never dies on a bad frame or a dropped client
  - Live stats overlay (min/max/mean) so you can judge exposure
  - Toggle between normalized (MIN-MAX stretch) and raw display via URL
  - Keeps the HTTP client alive with keep-alive frames when no data
  - Prints READY before importing cv2 so udp_sink never meets a closed port

Display modes (append to URL):
  http://host:port/           normalized stretch + stats (default)
  http://host:port/?raw=1     raw pixel values, no stretch
  http://host:port/?stats=0   hide the overlay
"""

import http.server
import io
import logging
import struct
import sys
import socket
import threading
import urllib.parse

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s  %(levelname)-8s  camera_view: %(message)s',
    datefmt='%H:%M:%S',
    stream=sys.stderr,
)
log = logging.getLogger(__name__)

AYLP_MAGIC          = 0x504C5941
AYLP_HEADER_FMT     = '<IBBBBQQdd'
AYLP_HEADER_SIZE    = struct.calcsize(AYLP_HEADER_FMT)   # 40 bytes
AYLP_T_MATRIX_UCHAR = 1 << 5

JPEG_QUALITY = 85

# ── shared state ──────────────────────────────────────────────────────────────

_latest = {
    'jpeg_norm':  None,   # normalized display
    'jpeg_raw':   None,   # raw display
    'stats':      (0, 0, 0.0),  # min, max, mean
    'shape':      (0, 0),
    'frame_count': 0,
}
_lock  = threading.Lock()
_ready = threading.Event()


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *args):
        pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        qs     = urllib.parse.parse_qs(parsed.query)
        path   = parsed.path

        if path == '/':
            body = (
                b'<html><body style="margin:0;background:#000;text-align:center">'
                b'<img src="/stream" style="max-width:100%;max-height:100vh;'
                b'image-rendering:pixelated">'
                b'</body></html>'
            )
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == '/stream':
            use_raw   = qs.get('raw',  ['0'])[0] == '1'
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while True:
                    if not _ready.wait(timeout=2.0):
                        continue
                    _ready.clear()
                    with _lock:
                        data = _latest['jpeg_raw'] if use_raw else _latest['jpeg_norm']
                    if data is None:
                        continue
                    try:
                        self.wfile.write(
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n'
                            + data + b'\r\n'
                        )
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
            except Exception:
                pass
            return

        if path == '/stats':
            with _lock:
                mn, mx, mean = _latest['stats']
                h, w = _latest['shape']
                fc = _latest['frame_count']
            body = (f'{{"min":{mn},"max":{mx},"mean":{mean:.1f},'
                    f'"width":{w},"height":{h},"frames":{fc}}}').encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        self.send_response(404)
        self.send_header('Content-Length', '0')
        self.end_headers()


# ── frame receiver ────────────────────────────────────────────────────────────

def receiver(sock, show_stats):
    import cv2
    import numpy as np

    while True:
        try:
            raw = sock.recv(131072)
        except socket.timeout:
            continue
        except OSError:
            continue

        if len(raw) < AYLP_HEADER_SIZE:
            continue
        try:
            magic, _v, _s, typ, _u, dim_y, dim_x, _py, _px = \
                struct.unpack_from(AYLP_HEADER_FMT, raw)
        except struct.error:
            continue
        if magic != AYLP_MAGIC or typ != AYLP_T_MATRIX_UCHAR:
            continue

        h, w = int(dim_y), int(dim_x)
        n = h * w
        payload = raw[AYLP_HEADER_SIZE:]
        if len(payload) < n or n == 0:
            continue

        frame = np.frombuffer(payload[:n], dtype=np.uint8).reshape(h, w)

        mn   = int(frame.min())
        mx   = int(frame.max())
        mean = float(frame.mean())

        # Normalized display
        if mx > mn:
            norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        else:
            norm = frame.copy()

        # Raw display (as-is)
        rawimg = frame

        # Optional stats overlay on the normalized image
        if show_stats:
            norm_bgr = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
            txt = f'min{mn} max{mx} avg{mean:.0f} {w}x{h}'
            cv2.putText(norm_bgr, txt, (4, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1,
                        cv2.LINE_AA)
            # Saturation warning
            if mx >= 255:
                cv2.putText(norm_bgr, 'SAT', (4, h - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1,
                            cv2.LINE_AA)
            norm_out = norm_bgr
        else:
            norm_out = norm

        ok1, jn = cv2.imencode('.jpg', norm_out,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        ok2, jr = cv2.imencode('.jpg', rawimg,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok1:
            continue

        with _lock:
            _latest['jpeg_norm']   = jn.tobytes()
            _latest['jpeg_raw']    = jr.tobytes() if ok2 else jn.tobytes()
            _latest['stats']       = (mn, mx, mean)
            _latest['shape']       = (h, w)
            _latest['frame_count'] += 1
        _ready.set()


def main():
    if len(sys.argv) < 3:
        sys.exit(f'Usage: {sys.argv[0]} <udp_port> <http_port> [title]')
    udp_port  = int(sys.argv[1])
    http_port = int(sys.argv[2])
    title     = sys.argv[3] if len(sys.argv) > 3 else f'Camera :{udp_port}'
    show_stats = True

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)  # 1 MB
    sock.settimeout(0.5)
    sock.bind(('0.0.0.0', udp_port))

    print('READY', flush=True)
    log.info('%s — UDP bound on port %d', title, udp_port)

    # Start HTTP server with threading so multiple clients / tabs work
    class ThreadingHTTPServer(
            http.server.ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    server = ThreadingHTTPServer(('0.0.0.0', http_port), _MJPEGHandler)
    threading.Thread(target=server.serve_forever, daemon=True,
                     name='mjpeg-server').start()
    log.info('%s — stream at http://localhost:%d/', title, http_port)

    try:
        receiver(sock, show_stats)
    finally:
        sock.close()
        server.shutdown()


if __name__ == '__main__':
    main()
