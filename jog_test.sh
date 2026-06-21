#!/bin/bash
# Clean jog test for motor 2 using literal JSON (no shell-escaping issues).
B="http://127.0.0.1:5000"
cs() { curl -sS "$B/motor/2/status" | grep -o '"current_speed":[0-9]*'; }

curl -sS -X POST "$B/motor/2/disable" -H 'Content-Type: application/json' -d '{"soft":false}' >/dev/null
sleep 1
echo "=== jog ON (cw) ==="
curl -sS -X POST "$B/motor/2/jog" -H 'Content-Type: application/json' -d '{"direction":"cw","on":true}' >/dev/null
for i in 1 2 3; do sleep 0.4; printf 'J%s ' "$i"; cs; done
echo "=== jog OFF (release) ==="
curl -sS -X POST "$B/motor/2/jog" -H 'Content-Type: application/json' -d '{"on":false}' >/dev/null
for i in 1 2 3 4 5; do sleep 0.4; printf 'R%s ' "$i"; cs; done
curl -sS -X POST "$B/motor/2/disable" -H 'Content-Type: application/json' -d '{"soft":false}' >/dev/null
echo "DONE"
