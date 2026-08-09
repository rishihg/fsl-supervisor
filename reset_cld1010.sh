#!/bin/bash
# reset_cld1010.sh — attempt a USB-level reset of the CLD1010 without
# needing to physically power-cycle the instrument.
#
# This toggles the device's 'authorized' flag off/on, which forces Linux
# to re-enumerate it — often enough to clear a wedged USBTMC session.
# It will NOT help if the instrument's own firmware has tripped a real
# protection fault (interlock, TEC, current limit) — in that case the
# front-panel ERROR tab and a stuck SYST:ERR? query mean you need to
# physically power-cycle the unit.
#
# Usage: ./reset_cld1010.sh

set -e

VID="1313"
PID="804f"

echo "Looking for CLD1010 (idVendor=$VID idProduct=$PID)..."

FOUND=""
for dev in /sys/bus/usb/devices/*/idVendor; do
    dir=$(dirname "$dev")
    if [ "$(cat "$dev" 2>/dev/null)" = "$VID" ] && \
       [ "$(cat "$dir/idProduct" 2>/dev/null)" = "$PID" ]; then
        FOUND="$dir"
        break
    fi
done

if [ -z "$FOUND" ]; then
    echo "CLD1010 not found on the USB bus. Is it powered on and plugged in?"
    exit 1
fi

PORT=$(basename "$FOUND")
echo "Found at USB port: $PORT"
echo "Resetting..."

echo 0 | sudo tee "$FOUND/authorized" > /dev/null
sleep 1
echo 1 | sudo tee "$FOUND/authorized" > /dev/null
sleep 2

echo "Reset complete. Checking device..."
if [ -e /dev/cld1010 ]; then
    echo "OK: /dev/cld1010 -> $(readlink /dev/cld1010)"
    echo
    echo "Querying error queue..."
    timeout 3 python3 -c "
import time
try:
    dev = open('/dev/cld1010', 'r+b', buffering=0)
    dev.write(b'SYST:ERR?\n')
    time.sleep(0.2)
    print(dev.read(256).decode(errors='replace'))
    dev.close()
except Exception as e:
    print(f'Still unresponsive: {e}')
    print('This means it is a real instrument fault (interlock/TEC/current')
    print('limit) or a firmware-level wedge — power-cycle the CLD1010 itself.')
"
else
    echo "WARNING: /dev/cld1010 did not come back."
    echo "Physically power-cycle the CLD1010."
fi
