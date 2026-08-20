#!/usr/bin/env bash
# One-time bootstrap for a fresh Raspberry Pi OS install.
# Sets up the led_app directory, Python dependencies, systemd service, UART for
# the SERVO42C bus, SPI0 for the WS2812 emotion-ring LEDs, and hardware PWM on
# GPIO12/13 for the axis 5/6 servos.
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
[ -f /tmp/emotion.html ] && mv /tmp/emotion.html "$APP_DIR/templates/emotion.html"
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

# 3b. Enable SPI0 for the WS2812 emotion-ring LEDs (data on MOSI/GPIO10).
#     Use spi0-1cs,no_miso so only CE0/GPIO8 is claimed: this keeps CE1/GPIO7
#     free (Motor 2 DIR) and frees MISO/GPIO9 (Motor 1 end-stop).
if ! grep -q '^dtparam=spi=on' "$CONFIG"; then
    sudo sed -i 's/^#dtparam=spi=on/dtparam=spi=on/' "$CONFIG"
    grep -q '^dtparam=spi=on' "$CONFIG" || echo 'dtparam=spi=on' | sudo tee -a "$CONFIG" >/dev/null
    echo "==> Enabled dtparam=spi=on in $CONFIG"
else
    echo "==> dtparam=spi=on already set"
fi
if ! grep -q '^dtoverlay=spi0-1cs,no_miso' "$CONFIG"; then
    echo 'dtoverlay=spi0-1cs,no_miso' | sudo tee -a "$CONFIG" >/dev/null
    echo "==> Added dtoverlay=spi0-1cs,no_miso to $CONFIG"
else
    echo "==> dtoverlay=spi0-1cs,no_miso already set"
fi

# 3c. Enable hardware PWM on GPIO12/13 for the axis 5/6 servos.
#     This creates the RP1 PWM0 controller (peripheral address 1f00098000)
#     that app.py auto-detects. Without it the servos are unavailable.
if ! grep -q '^dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' "$CONFIG"; then
    echo 'dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4' | sudo tee -a "$CONFIG" >/dev/null
    echo "==> Added dtoverlay=pwm-2chan (GPIO12/13 servo PWM) to $CONFIG"
else
    echo "==> dtoverlay=pwm-2chan already set"
fi

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
