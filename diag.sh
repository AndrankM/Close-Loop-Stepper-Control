#!/bin/bash
for c in /sys/class/pwm/pwmchip*; do
  echo "== $c =="
  echo -n 'npwm: '; cat "$c/npwm" 2>/dev/null
  echo -n 'device: '; readlink -f "$c/device" 2>/dev/null
done
echo "---APPLOG---"
journalctl -u led_app.service -n 40 --no-pager 2>&1 | grep -i -E 'pwm|servo|error|trace|chip|perm|hardwarepwm' | tail -20
echo DONE
