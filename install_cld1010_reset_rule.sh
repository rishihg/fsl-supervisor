#!/bin/bash
# Installs the udev rule that lets hst_supervisor's CLD1010 driver issue a
# real USB port reset (USBDEVFS_RESET) when the instrument's usbtmc
# endpoint wedges, instead of needing a manual reset_cld1010.sh run or a
# physical power-cycle. Must be run with sudo.
set -e

if [ "$EUID" -ne 0 ]; then
    echo "Run this with sudo: sudo bash $0"
    exit 1
fi

RULE_FILE=/etc/udev/rules.d/99-cld1010-reset.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1313", ATTRS{idProduct}=="804f", MODE="0666"' > "$RULE_FILE"
udevadm control --reload-rules
udevadm trigger

echo "Installed $RULE_FILE:"
cat "$RULE_FILE"
