#!/bin/bash
# Hardware PWM servo setup for Pi 5 (RP1). Run with sudo.
set -e

echo "=== Stop & disable broken pigpiod (no Pi 5 support) ==="
systemctl disable --now pigpiod 2>/dev/null || true
echo "pigpiod disabled"

echo "=== Install rpi-hardware-pwm ==="
pip3 install rpi-hardware-pwm --break-system-packages 2>&1 | tail -3 || \
  pip3 install rpi-hardware-pwm 2>&1 | tail -3

echo "=== Enable GPIO 12/13 hardware PWM overlay ==="
CFG=/boot/firmware/config.txt
if ! grep -q "pwm-2chan,pin=12" "$CFG"; then
  echo "" >> "$CFG"
  echo "# Hardware PWM on GPIO 12 (chan0) & GPIO 13 (chan1) for servos" >> "$CFG"
  echo "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4" >> "$CFG"
  echo "overlay added to $CFG"
else
  echo "overlay already present"
fi

echo "=== Deploy app.py ==="
cp /tmp/app.py /home/andpi5/led_app/app.py
chown andpi5:andpi5 /home/andpi5/led_app/app.py
echo "app.py deployed"

echo "=== Current config.txt PWM lines ==="
grep -n pwm "$CFG" || true

echo "DONE - REBOOT REQUIRED to load the PWM overlay"
