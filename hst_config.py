"""
hst_config.py — HST terminal hardware configuration.

Edit this file when hardware changes. The TUI imports everything from here.
"""
VIEWER_SCRIPT = '/home/qshanty/supervisor/camera_view2.py'
ANYLOOP_BINARY = '/home/qshanty/bin/anyloop'
CAMERA_CONFIGS = {
    0: ('/home/qshanty/confocal.json', 54741, 'CONFOCAL', 9290),
    1: ('/home/qshanty/fiber.json',    54742, 'FIBER',    9663),
}



# ── NanoMax fiber stage (Tic T834, x/y/z) ────────────────────────────────────

FIBER_TIC_SERIALS: dict[str, str] = {
    'x': '00499707',
    'y': '00499742',
    'z': '00499722',
}

FIBER_TIC_STEP_SIZE = 5   # microsteps per keypress — tune per axis

# ── Piezo steering mirror (MDT693B, x/y) ─────────────────────────────────────

MDT693B_PORT     = '/dev/mdt693b'
MDT693B_STEP_V   = 1.0    # volts per keypress — tune to taste

# ── flip mounts (Tic T834) ────────────────────────────────────────────────────
# Add serials here once flip mount Tics are connected.
# FLIP_TIC_SERIALS: dict[str, str] = {
#     'fm1': '00XXXXXX',    
# }
# FLIP_POSITIONS: dict[str, tuple[int, int]] = {
#     'fm1': (0, 1000),   # (position_in, position_out) in microsteps
# }

# ── Arduino Leonardo auxiliary controller ─────────────────────────────────────

ARDUINO_PORT = '/dev/arduino'

# ── CLD1010LP laser controller ────────────────────────────────────────────────

CLD1010_DEVICE  = '/dev/cld1010'
CLD1010_CURRENT = 0.110   # amps — last known setpoint, update as needed

# ── Flip mount ────────────────────────────────────────────────────────────────
FLIP_MOUNT_SERIAL    = '00499725'
FLIP_MOUNT_DEGREES   = 90
FLIP_MOUNT_DIRECTION = 1

RIGOL_DEVICE = '/dev/rigol_dg4202'

QED_VNC_HOST = '10.2.246.46'

# ── Toptica DLC pro tunable diode laser ───────────────────────────────────────

DLCPRO_HOST   = '10.2.245.161'
DLCPRO_SERIAL = 'DLC PRO_041413'
