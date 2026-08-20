#!/usr/bin/env bash
# One-time bootstrap for a fresh Raspberry Pi OS install.
# Sets up the led_app directory, Python dependencies, systemd service, and UART
# for the SERVO42C bus.
set -e

USER_NAME="$(whoami)"
HOME_DIR="$HOME"
APP_DIR="$HOME_DIR/led_app"
PY="$(command -v python3)"

echo "==> User=$USER_NAME  AppDir=$APP_DIR  Python=$PY"

# 1. App directory + move uploaded files into place
mkdir -p "$APP_DIR/templates"
[ -f /tmp/app.py ] && mv /tmp/app.py "$APP_DIR/app.py"
[ -f /tmp/index.html ] && mv /tmp/index.html "$APP_DIR/templates/index.html"
echo "==> App files in place:"
ls -l "$APP_DIR" "$APP_DIR/templates"

# 1b. Python dependencies (system-wide, since the service runs /usr/bin/python3)
#     Core deps are required; the rest degrade to no-ops if missing.
echo "==> Installing Python dependencies (apt) ..."
sudo apt-get update
sudo apt-get install -y \
    python3-flask \
    python3-serial \
    python3-spidev \
    python3-numpy
# rpi-hardware-pwm (axis 5/6 servos) is pip-only; optional, never fatal.
if ! python3 -c 'import rpi_hardware_pwm' 2>/dev/null; then
    sudo pip3 install --break-system-packages rpi-hardware-pwm \
        || echo "==> WARN: rpi-hardware-pwm not installed (axis 5/6 servos disabled)"
fi

# 2. Enable UART on the GPIO header (TXD GPIO14 / RXD GPIO15)
CONFIG=/boot/firmware/config.txt
if ! grep -q '^enable_uart=1' "$CONFIG"; then
    echo 'enable_uart=1' | sudo tee -a "$CONFIG" >/dev/null
    echo "==> Added enable_uart=1 to $CONFIG"
else
    echo "==> enable_uart=1 already set"
fi

# 3. Free the serial port from the login console
CMDLINE=/boot/firmware/cmdline.txt
if grep -q 'console=serial0,[0-9]*' "$CMDLINE"; then
    sudo sed -i 's/console=serial0,[0-9]* //' "$CMDLINE"
    echo "==> Removed serial console from $CMDLINE"
else
    echo "==> serial console already removed"
fi
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable --now serial-getty@ttyS0.service 2>/dev/null || true

# 4. systemd service (runs system python3 directly; all deps are system-wide)
SERVICE=/etc/systemd/system/led_app.service
sudo tee "$SERVICE" >/dev/null <<EOF
[Unit]
Description=Closed-Loop Stepper Control Flask app
After=network.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$APP_DIR
ExecStart=$PY $APP_DIR/app.py
Restart=on-failure
RestartSec=3
Nice=10
CPUQuota=70%

[Install]
WantedBy=multi-user.target
EOF
echo "==> Wrote $SERVICE"

sudo systemctl daemon-reload
sudo systemctl enable led_app.service
echo "==> Service enabled. (Will start on boot; UART change needs a reboot.)"
echo "==> Bootstrap complete."
