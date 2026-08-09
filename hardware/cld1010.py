"""
hardware/cld1010.py — Thorlabs CLD1010LP laser diode controller driver.

Communicates via the usbtmc device directly (no NI-VISA/libusb needed).

Make permission permanent with:
    echo 'SUBSYSTEM=="usbtmc", ATTRS{idVendor}=="1313", MODE="0666"' | \\
        sudo tee /etc/udev/rules.d/99-cld1010.rules
    sudo udevadm control --reload-rules
Then replug the CLD1010.

Also grant access to the *raw* USB device node (separate from the usbtmc
character device above) so this driver can issue a port-level reset when
the instrument wedges, without needing sudo at runtime:
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1313", ATTRS{idProduct}=="804f", MODE="0666"' | \\
        sudo tee /etc/udev/rules.d/99-cld1010-reset.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
(No replug needed for this one — udevadm trigger re-applies permissions
to the device that's already attached.)

Robustness notes:
  - Reads use a plain blocking read relying on the kernel usbtmc driver's
    own read timeout, not select() — select() can falsely report "not
    ready" on this transport even when a response is pending (confirmed
    on the Rigol DG4202, same usbtmc driver), which caused false
    TimeoutErrors.
  - One failed command triggers a single automatic reopen-and-retry via
    _reopen(). If that also fails, the exception propagates so the
    caller can show NOT CONNECTED rather than silently hanging or
    corrupting state.
  - _reopen() does a REAL USB port reset (USBDEVFS_RESET), not just a
    close()/open() of the Python file object. This matters: the failure
    mode actually seen in the field is the kernel logging
    "usbtmc ...: Unable to send data, error -110" (ETIMEDOUT) while the
    device stays enumerated — the bulk endpoint itself is wedged, and
    cycling the file descriptor alone does not clear that (confirmed:
    restarting the whole supervisor process reliably failed to
    reconnect while this condition was active, because the fresh
    process's fd-only reopen hit the same wedged endpoint). A port
    reset is the software equivalent of unplug/replug and does clear
    it, without needing to physically power-cycle the instrument. See
    reset_cld1010.sh for the same fix at the shell level.
  - Run only ONE process against this device at a time. Two processes
    opening the same usbtmc device concurrently WILL interleave
    commands/responses and look like random hardware failures.
"""

import fcntl
import glob
import logging
import os
import time

log = logging.getLogger(__name__)

_USB_VENDOR_ID  = '1313'   # Thorlabs
_USB_PRODUCT_ID = '804f'   # CLD1010LP
_USBDEVFS_RESET = 21780    # linux/usbdevice_fs.h: _IO('U', 20)


def _find_usb_device_node(vendor_id=_USB_VENDOR_ID, product_id=_USB_PRODUCT_ID):
    """Locate /dev/bus/usb/BBB/DDD for the CLD1010 by scanning sysfs for
    idVendor/idProduct, the same way `usbreset` does — bus/device numbers
    are reassigned on every re-enumeration so they can't be hardcoded."""
    for vendor_path in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        devdir = os.path.dirname(vendor_path)
        try:
            with open(vendor_path) as f:
                if f.read().strip() != vendor_id:
                    continue
            with open(os.path.join(devdir, 'idProduct')) as f:
                if f.read().strip() != product_id:
                    continue
            with open(os.path.join(devdir, 'busnum')) as f:
                busnum = int(f.read().strip())
            with open(os.path.join(devdir, 'devnum')) as f:
                devnum = int(f.read().strip())
            return f'/dev/bus/usb/{busnum:03d}/{devnum:03d}'
        except (OSError, ValueError):
            continue
    return None


def _usb_port_reset(vendor_id=_USB_VENDOR_ID, product_id=_USB_PRODUCT_ID):
    """Issue a real USB port reset. Returns True if the ioctl was sent
    successfully, False if it couldn't even be attempted (device node
    not found, or no permission — e.g. the 99-cld1010-reset.rules udev
    rule described in this module's docstring isn't installed). Never
    raises: a failed reset just means the caller falls back to the old
    behaviour (plain fd reopen) instead of blowing up the retry path."""
    node = _find_usb_device_node(vendor_id, product_id)
    if not node:
        log.warning('CLD1010: could not locate USB device node for reset '
                    '(vendor=%s product=%s) — is it still plugged in?',
                    vendor_id, product_id)
        return False
    try:
        fd = os.open(node, os.O_WRONLY)
    except OSError as exc:
        log.warning('CLD1010: cannot open %s for USB reset (%s) — check '
                    'the 99-cld1010-reset.rules udev rule is installed',
                    node, exc)
        return False
    try:
        fcntl.ioctl(fd, _USBDEVFS_RESET, 0)
        return True
    except OSError as exc:
        log.warning('CLD1010: USBDEVFS_RESET on %s failed: %s', node, exc)
        return False
    finally:
        os.close(fd)


class CLD1010:
    """Driver for the Thorlabs CLD1010LP laser diode controller.

    Args:
        device:  Path to the usbtmc device, e.g. '/dev/cld1010'.
        timeout: Seconds to wait for a response before raising TimeoutError.
    """

    def __init__(self, device: str = '/dev/usbtmc0', timeout: float = 2.0):
        self._device_path = device
        self._timeout = timeout
        self._dev = open(device, 'r+b', buffering=0)
        # Sync state
        idn = self._ask('*IDN?')
        log.info('CLD1010 connected: %s', idn)
        self._laser_on   = self._ask('OUTP:STAT?').strip() in ('1', 'ON')
        self._current_sp = float(self._ask('SOUR:CURR:LEV?'))
        log.info('CLD1010 state: laser=%s current_sp=%.4f A',
                 self._laser_on, self._current_sp)

    # ── low-level ─────────────────────────────────────────────────────────────

    def _reopen(self):
        log.warning('CLD1010: reopening %s after a communication error',
                    self._device_path)
        try:
            self._dev.close()
        except Exception:
            pass

        if _usb_port_reset():
            log.warning('CLD1010: issued a USB port reset to clear a '
                        'wedged endpoint (a plain fd reopen alone cannot '
                        'do this — see "Unable to send data, error -110" '
                        'in dmesg/journalctl -k when this happens)')
            wait_s, attempts = 0.5, 10   # give the kernel time to rebind
        else:
            wait_s, attempts = 0.2, 1    # unchanged fallback behaviour

        last_exc = None
        for _ in range(attempts):
            time.sleep(wait_s)
            try:
                self._dev = open(self._device_path, 'r+b', buffering=0)
                return
            except OSError as exc:
                last_exc = exc
        raise last_exc

    def _send_raw(self, cmd: str) -> None:
        self._dev.write((cmd + '\n').encode())

    def _read_raw(self) -> str:
        # NOTE: deliberately NOT using select() here. The kernel's usbtmc
        # driver does not reliably implement poll()/select() readiness for
        # these instruments — select() can report "not ready" even when a
        # response is actually pending, causing false timeouts (confirmed
        # on the Rigol DG4202, same usbtmc transport as this device). A
        # plain blocking read, relying on the kernel's own usbtmc read
        # timeout, is the proven-working approach.
        data = self._dev.read(256)
        if not data:
            raise TimeoutError('CLD1010: empty response')
        return data.decode('utf-8', errors='replace').strip()

    def _log_instrument_error(self):
        """Best-effort: ask the instrument what actually went wrong, so the
        log shows the real fault (interlock/TEC/current limit/etc) instead
        of just 'timed out'. Never raises."""
        try:
            self._send_raw('SYST:ERR?')
            time.sleep(0.1)
            err = self._read_raw()
            if err and not err.startswith('+0'):
                log.error('CLD1010 instrument error queue: %s', err)
        except Exception:
            pass  # device may be too wedged to even answer this

    def _send(self, cmd: str) -> None:
        """Send with one automatic reopen-and-retry on failure."""
        try:
            self._send_raw(cmd)
        except (OSError, TimeoutError) as exc:
            log.warning('CLD1010: write %r failed (%s)', cmd, exc)
            self._log_instrument_error()
            self._reopen()
            self._send_raw(cmd)

    def _ask(self, cmd: str) -> str:
        """Send+read with one automatic reopen-and-retry on failure."""
        try:
            self._send_raw(cmd)
            time.sleep(0.1)
            return self._read_raw()
        except (OSError, TimeoutError) as exc:
            log.warning('CLD1010: command %r failed (%s); retrying once',
                       cmd, exc)
            self._log_instrument_error()
            self._reopen()
            self._send_raw(cmd)
            time.sleep(0.1)
            return self._read_raw()

    # ── laser output ──────────────────────────────────────────────────────────

    @property
    def laser_on(self) -> bool:
        return self._laser_on

    def set_laser(self, on: bool) -> None:
        """Enable or disable the laser output."""
        self._send(f'OUTP:STAT {"ON" if on else "OFF"}')
        time.sleep(0.2)
        resp = self._ask('OUTP:STAT?').strip()
        self._laser_on = resp in ('1', 'ON')
        log.info('CLD1010 laser -> %s (confirmed: %s)',
                 'ON' if on else 'OFF', resp)

    def laser_toggle(self) -> None:
        self.set_laser(not self._laser_on)

    # ── current setpoint ──────────────────────────────────────────────────────

    @property
    def current_sp(self) -> float:
        """Current setpoint in amps."""
        return self._current_sp

    def set_current(self, amps: float) -> None:
        """Set the laser current in amps."""
        self._send(f'SOUR:CURR:LEV {amps:.6f}')
        time.sleep(0.1)
        self._current_sp = float(self._ask('SOUR:CURR:LEV?'))
        log.info('CLD1010 current setpoint -> %.4f A', self._current_sp)

    # ── measurements ─────────────────────────────────────────────────────────

    def measure_current(self) -> float:
        """Read the actual laser current in amps."""
        return float(self._ask('MEAS:CURR?'))

    def measure_temperature(self) -> float:
        """Read the actual laser diode temperature in Celsius."""
        return float(self._ask('MEAS:TEMP?'))

    def measure_voltage(self) -> float:
        """Read the actual laser voltage in volts."""
        return float(self._ask('MEAS:VOLT?'))

    # ── TEC ───────────────────────────────────────────────────────────────────

    def set_tec(self, on: bool) -> None:
        """Enable or disable the TEC output."""
        self._send(f'OUTP2:STAT {"ON" if on else "OFF"}')
        log.info('CLD1010 TEC -> %s', 'ON' if on else 'OFF')

    # ── status ────────────────────────────────────────────────────────────────

    def status(self) -> dict:
        return {
            'laser_on':    self._laser_on,
            'current_sp':  self._current_sp,
            'current_act': self.measure_current(),
        }

    def shutdown(self) -> None:
        """Safe state: laser off."""
        try:
            self.set_laser(False)
            self._dev.close()
        except Exception:
            pass
