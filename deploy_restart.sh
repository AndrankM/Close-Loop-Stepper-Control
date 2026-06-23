#!/bin/bash
cp /tmp/app.py /home/andpi5/led_app/app.py
chown andpi5:andpi5 /home/andpi5/led_app/app.py
systemctl restart led_app.service
sleep 3
echo "=== service ==="
systemctl is-active led_app.service
echo "=== servo status ==="
curl -s http://127.0.0.1:5000/servo/status
echo
echo "=== app log (servo) ==="
journalctl -u led_app.service -n 30 --no-pager 2>&1 | grep -i -E 'servo|pwm|chip|error|trace' | tail -15
echo DONE
