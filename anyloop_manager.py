"""
anyloop_manager.py — subprocess lifecycle and UDP IPC for anyloop.

AnyloopProcess  — spawns/stops the anyloop binary.
TelemetryReceiver — background thread that parses AYLP packets from
                    anyloop's udp_sink and exposes the latest reading.
CommandSender   — sends command vectors to anyloop's udp_source device.
"""

import logging
import socket
import struct
import subprocess
import threading
from dataclasses import dataclass
from typing import Optional

log = logging.getLogger(__name__)

# ── AYLP wire format (libaylp/anyloop.h, __attribute__((packed))) ──────────
# uint32  magic
# uint8   version
# uint8   status
# uint8   type
# uint8   units
# uint64  log_dim.y
# uint64  log_dim.x
# double  pitch.y
# double  pitch.x
AYLP_MAGIC = 0x504C5941
AYLP_HEADER_FMT = '<IBBBBQQdd'
AYLP_HEADER_SIZE = struct.calcsize(AYLP_HEADER_FMT)   # 40 bytes

# aylp_type flags
AYLP_T_VECTOR = 1 << 2   # 4
AYLP_T_MATRIX = 1 << 3   # 8

# aylp_status flags
AYLP_DONE = 1 << 0


@dataclass
class AylpPacket:
    """Parsed AYLP telemetry packet."""
    status: int
    type: int
    units: int
    log_dim_y: int
    log_dim_x: int
    data: list       # payload as list of floats (for vector/matrix types)


def _parse_aylp(raw: bytes) -> Optional[AylpPacket]:
    if len(raw) < AYLP_HEADER_SIZE:
        log.warning('Short AYLP packet: %d bytes', len(raw))
        return None
    magic, version, status, typ, units, dim_y, dim_x, pitch_y, pitch_x = \
        struct.unpack_from(AYLP_HEADER_FMT, raw)
    if magic != AYLP_MAGIC:
        log.warning('Bad AYLP magic: 0x%08X', magic)
        return None
    payload = raw[AYLP_HEADER_SIZE:]
    n = dim_y * dim_x
    if typ & AYLP_T_VECTOR:
        expected = n * 8
        if len(payload) < expected:
            log.warning('Truncated vector payload: %d < %d', len(payload), expected)
            return None
        values = list(struct.unpack_from(f'<{n}d', payload))
    elif typ & AYLP_T_MATRIX:
        # parse as flat list of doubles
        n_doubles = len(payload) // 8
        values = list(struct.unpack_from(f'<{n_doubles}d', payload))
    else:
        values = []
    return AylpPacket(status=status, type=typ, units=units,
                      log_dim_y=dim_y, log_dim_x=dim_x, data=values)


class TelemetryReceiver:
    """Listens on a UDP port for AYLP packets from anyloop's udp_sink.

    Runs in a background daemon thread.  SO_REUSEPORT is set so the Julia
    visualiser can bind the same port simultaneously.
    """

    def __init__(self, port: int, rcvbuf: int = 512):
        self.port = port
        self.rcvbuf = rcvbuf
        self._latest: Optional[AylpPacket] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f'telemetry:{self.port}')
        self._thread.start()
        log.debug('TelemetryReceiver started on port %d', self.port)

    def stop(self):
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def get(self) -> Optional[AylpPacket]:
        """Return the most recently received packet, or None."""
        with self._lock:
            return self._latest

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
        sock.settimeout(0.5)
        sock.bind(('127.0.0.1', self.port))
        self._sock = sock
        while not self._stop.is_set():
            try:
                raw = sock.recv(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            pkt = _parse_aylp(raw)
            if pkt is not None:
                with self._lock:
                    self._latest = pkt
        log.debug('TelemetryReceiver stopped on port %d', self.port)


class CommandSender:
    """Sends command vectors to anyloop's udp_source device.

    Packet format: n native-endian doubles, no header — matches what
    udp_source_proc() expects in devices/udp_source.c.
    """

    def __init__(self, port: int, n: int):
        self.port = port
        self.n = n
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, *values: float):
        if len(values) != self.n:
            raise ValueError(f'Expected {self.n} values, got {len(values)}')
        self._sock.sendto(
            struct.pack(f'<{self.n}d', *values),
            ('127.0.0.1', self.port)
        )

    def close(self):
        self._sock.close()


class AnyloopProcess:
    """Manages an anyloop subprocess.

    stdout and stderr are optionally captured to a log file; if no log file
    is specified, stdout is discarded and stderr is passed through.
    """

    def __init__(self, binary: str = '/home/fsl/bin/anyloop'):
        self.binary = binary
        self._proc: Optional[subprocess.Popen] = None
        self._logfile = None

    def start(self, config: str, log_file: Optional[str] = '/home/fsl/anyloop.log'):
        if self.running:
            raise RuntimeError('anyloop is already running')
        if log_file:
            self._logfile = open(log_file, 'a')
            stdout = self._logfile
            stderr = self._logfile
        else:
            stdout = subprocess.DEVNULL
            stderr = subprocess.DEVNULL
        self._proc = subprocess.Popen(
            [self.binary, config],
            stdout=stdout,
            stderr=stderr,
        )
        log.info('anyloop started: pid=%d config=%s', self._proc.pid, config)

    def stop(self, timeout: float = 2.0):
        if not self.running:
            return
        self._proc.terminate()
        try:
            self._proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            log.warning('anyloop did not terminate cleanly; killing')
            self._proc.kill()
            self._proc.wait()
        if self._logfile:
            self._logfile.close()
            self._logfile = None
        log.info('anyloop stopped')

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def pid(self) -> Optional[int]:
        return self._proc.pid if self._proc else None
