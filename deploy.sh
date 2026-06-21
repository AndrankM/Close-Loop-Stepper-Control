#!/bin/bash
set -e
echo "=== pigpio python check ==="
python3 -c 'import pigpio; print("pigpio module OK")' 2>&1 || echo "pigpio module MISSING"
echo "=== pigpiod status ==="
systemctl is-active pigpiod 2>&1 || true
echo "=== services matching app ==="
systemctl list-units --type=service --all 2>/dev/null | grep -i -E 'led|flask|robot|app' || echo "no matching service"
echo "=== deploy file ==="
cp /tmp/app.py /home/andpi5/led_app/app.py
echo "copied to /home/andpi5/led_app/app.py"
echo "DONE"
