#!/bin/bash
# Deploy the digital-twin Pose & Send release: app.py + index.html (robot assets
# are uploaded separately via SFTP). Run on the Pi as: sudo bash /tmp/deploy_twin_restart.sh
set -e
cp /tmp/app.py /home/andpi5/led_app/app.py
cp /tmp/index.html /home/andpi5/led_app/templates/index.html
chown andpi5:andpi5 /home/andpi5/led_app/app.py /home/andpi5/led_app/templates/index.html
# Stale base mesh is no longer referenced by the URDF; remove if present.
rm -f /home/andpi5/led_app/static/robot/meshes/base_link.STL
chown -R andpi5:andpi5 /home/andpi5/led_app/static/robot
systemctl restart led_app.service
sleep 3
echo "=== service ==="
systemctl is-active led_app.service
echo "=== /twin/config ==="
curl -s http://127.0.0.1:5000/twin/config
echo
echo "=== robot assets on disk ==="
ls -la /home/andpi5/led_app/static/robot/ /home/andpi5/led_app/static/robot/meshes/
echo "=== recent log ==="
journalctl -u led_app.service -n 25 --no-pager 2>&1 | tail -15
echo DONE
