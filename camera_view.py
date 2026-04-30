#!/home/fsl/supervisor/.venv/bin/python3
"""
camera_view.py — serve AYLP camera frames as an MJPEG stream over HTTP.

Usage: camera_view.py <udp_port> <http_port> [title]

Open  http://<host>:<http_port>/  in a browser to view the stream.
For remote access, forward the port first:
    ssh -L <http_port>:localhost:<http_port> <rooftop-host>
then open  http://localhost:<http_port>/

The UDP socket is bound before cv2/numpy are imported so that anyloop's
udp_sink never sees an unbound port regardless of startup time.
"""

# ── bind UDP socket first (only stdlib needed here) ───────────────────────────
import http.server
import io
import logging
import struct
import sys
import socket
import threading

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
AYLP_T_MATRIX_UCHAR = 1 << 5                             # = 32

DISPLAY_SCALE = 1    # no upscale; browser CSS handles display scaling
JPEG_QUALITY  = 85


# ── shared state between receiver and HTTP handler ────────────────────────────

_latest_jpeg: bytes | None = None
_jpeg_lock  = threading.Lock()
_jpeg_ready = threading.Event()


# ── MJPEG HTTP handler ────────────────────────────────────────────────────────

class _MJPEGHandler(http.server.BaseHTTPRequestHandler):

    def log_message(self, *args):
        pass  # suppress per-request access log

    def do_GET(self):
        if self.path == '/':
            body = (
                b'<html><body style="margin:0;background:#000">'
                b'<img src="/stream" style="max-width:100%;max-height:100vh">'
                b'</body></html>'
            )
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path != '/stream':
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                if not _jpeg_ready.wait(timeout=2.0):
                    continue   # no frame yet — keep client alive
                _jpeg_ready.clear()
                with _jpeg_lock:
                    data = _latest_jpeg
                if data is None:
                    continue
                try:
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n'
                        + data + b'\r\n'
                    )
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
        except Exception:
            pass


def main():
    if len(sys.argv) < 3:
        sys.exit(f'Usage: {sys.argv[0]} <udp_port> <http_port> [title]')
    udp_port  = int(sys.argv[1])
    http_port = int(sys.argv[2])
    title     = sys.argv[3] if len(sys.argv) > 3 else f'Camera :{udp_port}'

    # ── bind UDP socket before importing cv2/numpy ───────────────────────────
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)  # ~2 frames
    sock.settimeout(0.5)
    sock.bind(('0.0.0.0', udp_port))

    # signal to CameraViewer.open() that the socket is ready
    print('READY', flush=True)
    log.info('%s — UDP bound on port %d', title, udp_port)

    # ── now import the slow modules ───────────────────────────────────────────
    import cv2
    import numpy as np

    # ── start MJPEG HTTP server in background thread ──────────────────────────
    server = http.server.HTTPServer(('0.0.0.0', http_port), _MJPEGHandler)
    threading.Thread(target=server.serve_forever, daemon=True,
                     name='mjpeg-server').start()
    log.info('%s — stream at http://localhost:%d/', title, http_port)

    # ── receive frames and encode as JPEG ─────────────────────────────────────
    global _latest_jpeg
    try:
        while True:
            try:
                raw = sock.recv(65535)
            except socket.timeout:
                continue

            if len(raw) < AYLP_HEADER_SIZE:
                continue
            magic, _ver, _status, typ, _units, dim_y, dim_x, _py, _px = \
                struct.unpack_from(AYLP_HEADER_FMT, raw)
            if magic != AYLP_MAGIC or typ != AYLP_T_MATRIX_UCHAR:
                continue

            n = int(dim_y) * int(dim_x)
            payload = raw[AYLP_HEADER_SIZE:]
            if len(payload) < n:
                continue

            frame = np.frombuffer(payload[:n], dtype=np.uint8).reshape(
                int(dim_y), int(dim_x)
            )
            disp = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            disp = cv2.resize(
                disp,
                (int(dim_x) * DISPLAY_SCALE, int(dim_y) * DISPLAY_SCALE),
                interpolation=cv2.INTER_NEAREST,
            )
            ok, jpeg = cv2.imencode(
                '.jpg', disp, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if not ok:
                continue

            with _jpeg_lock:
                _latest_jpeg = jpeg.tobytes()
            _jpeg_ready.set()

    finally:
        sock.close()
        server.shutdown()


if __name__ == '__main__':
    main()
