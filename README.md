# Closed-Loop Stepper Control

A Raspberry Pi 5 web application for driving a **NEMA 17** stepper motor through an
**MKS SERVO42C** driver, with **live closed-loop feedback** from the SERVO42C's
built-in magnetic encoder over UART.

The Flask backend controls the motor via GPIO step/direction pulses and reads the
encoder over the serial port. A single-page dashboard provides motion control plus
a live encoder readout — angle dial, rotation counter, and a real-time chart.

## Features

- **Motor control** — enable/disable, direction (CW/CCW), speed (steps/s), and
  target RPM.
- **Acceleration ramp** — smooth ramp toward the target speed with a configurable
  acceleration (steps/s²).
- **Soft stop & emergency stop** — ramp-down disable or immediate de-energize.
- **RPM from geometry** — converts steps/s ↔ RPM using full steps/rev and
  microstepping settings.
- **Live encoder feedback (closed loop)** — reads the SERVO42C magnetic encoder
  over UART and displays:
  - Shaft angle dial (0–360°)
  - **Turns-since-reset** rotation counter with a Reset (tare) button
  - Total revolutions and total angle
  - Raw encoder value
  - A rolling 30-second time-series chart (angle / total angle / revolutions)
- **Graceful degradation** — runs without GPIO or serial hardware present
  (e.g. for local development) and reports availability in the API.

## Hardware

### Motion control (step/dir)

| Signal | Pi GPIO (BCM) | SERVO42C |
| ------ | ------------- | -------- |
| EN     | GPIO 17       | En (active-LOW) |
| STP    | GPIO 27       | Step     |
| DIR    | GPIO 22       | Dir      |
| GND    | GND           | GND      |

### Encoder feedback (UART / TTL)

| Pi | Pin | SERVO42C |
| -- | --- | -------- |
| TXD (GPIO 14) | physical pin 8  | Rx |
| RXD (GPIO 15) | physical pin 10 | Tx |
| GND           | —               | G  |

Leave the SERVO42C `3V3` pin unconnected.

SERVO42C UART defaults used here: **9600 baud**, **address `0xE0`** (OLED address
slot `0`). Set the matching baud and address on the driver's OLED menu.

> On the **Raspberry Pi 5**, the GPIO 14/15 UART must be enabled and freed from the
> serial console — see Setup below. Without this, `/dev/serial0` points at the
> dedicated 3-pin debug connector, not the GPIO header.

## Setup (Raspberry Pi 5)

1. **Enable the GPIO UART** and free it from the serial login console:

   ```bash
   # Enable the PL011 UART on GPIO 14/15
   echo 'enable_uart=1' | sudo tee -a /boot/firmware/config.txt

   # Disable the serial console getty and remove it from the kernel cmdline
   sudo systemctl disable --now serial-getty@ttyAMA0.service
   sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt

   sudo reboot
   ```

   After reboot, `/dev/serial0` should point at the GPIO UART (`ttyAMA0`).

2. **Create the virtual environment.** On the Pi, `lgpio`/`gpiozero` are best used
   from system packages (building `lgpio` via pip needs `swig`), so create the venv
   with system site packages and install Flask into it:

   ```bash
   cd ~/led_app
   python3 -m venv --system-site-packages venv
   ./venv/bin/pip install flask pyserial
   ```

3. **Run the app:**

   ```bash
   ./venv/bin/python app.py
   ```

   The dashboard is served on `http://<pi-ip>:5000`.

### Run as a service (optional)

Install a `systemd` unit so the app starts on boot:

```ini
# /etc/systemd/system/led_app.service
[Unit]
Description=Stepper control web app
After=network.target

[Service]
User=andpi5
WorkingDirectory=/home/andpi5/led_app
ExecStart=/home/andpi5/led_app/venv/bin/python /home/andpi5/led_app/app.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now led_app
```

## Configuration

The serial settings can be overridden with environment variables:

| Variable      | Default        | Description                       |
| ------------- | -------------- | --------------------------------- |
| `SERVO_UART`  | `/dev/serial0` | Serial device path                |
| `SERVO_BAUD`  | `9600`         | UART baud rate                    |
| `SERVO_ADDR`  | `0xe0`         | SERVO42C address byte             |

## API

| Method | Route              | Description                                 |
| ------ | ------------------ | ------------------------------------------- |
| GET    | `/`                | Dashboard UI                                |
| POST   | `/motor/enable`    | Energize and start the motor                |
| POST   | `/motor/disable`   | Stop the motor (`{"soft": true\|false}`)    |
| POST   | `/motor/direction` | Set direction (`{"direction": "cw\|ccw"}`)  |
| POST   | `/motor/speed`     | Set speed in steps/s                        |
| POST   | `/motor/rpm`       | Set target RPM                              |
| POST   | `/motor/geometry`  | Set full steps/rev and microstepping        |
| POST   | `/motor/accel`     | Set acceleration (steps/s²)                 |
| GET    | `/motor/status`    | Motor status JSON                           |
| GET    | `/motor/encoder`   | Encoder reading JSON                        |

### Encoder response

```json
{
  "available": true,
  "error": null,
  "carry": 148,
  "value": 11736,
  "counts": 9711064,
  "angle_deg": 64.47,
  "total_angle_deg": 53344.47,
  "revolutions": 148.1791
}
```

The encoder reports a 16-bit `value` (0–0xFFFF over one revolution) and a signed
`carry` that increments/decrements each full turn, so the total count
(`carry * 65536 + value`) tracks absolute position across many rotations.

## Project structure

```
led_app/
  app.py               Flask backend: motor control + encoder reader
  templates/
    index.html         Dashboard UI (motion controls + live encoder)
  redeploy.ps1         Deploy script (scp + restart service over SSH)
blink_led.py           Standalone onboard LED blink example
```

## Deployment

`led_app/redeploy.ps1` copies `app.py` and `index.html` to the Pi and restarts the
service:

```powershell
./led_app/redeploy.ps1 -PiHost 192.168.0.103 -PiUser andpi5
```

## License

MIT
