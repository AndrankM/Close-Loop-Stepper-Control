# Closed-Loop Stepper Control

A Raspberry Pi 5 web application for driving **four NEMA 17** stepper motors through
**MKS SERVO42C** drivers, with **live closed-loop feedback** from each driver's
built-in magnetic encoder over a shared UART bus.

The Flask backend controls the motors via GPIO step/direction signals and reads the
encoders over the serial port. A single-page dashboard provides per-motor motion
control plus a live encoder readout — angle dial, rotation counter, and a real-time
chart — for each motor.

Most motors are direct-drive; motor 2 runs through a **5:1 planetary reducer**, and
the app reports its RPM and rotation relative to the geared output shaft.

## Features

- **Four independent motors** — each with its own control card and live encoder card.
- **Motor control** — enable/disable, direction (CW/CCW), speed (steps/s), and
  target RPM.
- **Hardware-timed step pulses** — step signals are generated with hardware PWM
  (frequency = step rate), giving accurate, linear speed control up to 6000 steps/s.
- **Acceleration ramp** — smooth ramp toward the target speed with a configurable
  acceleration (steps/s²).
- **Gear-ratio aware** — per-motor gear ratios (e.g. motor 2's 5:1 planetary
  reducer) are applied so RPM and encoder turns/angle are reported at the
  **output shaft**.
- **Soft stop & emergency stop** — ramp-down disable or immediate de-energize.
- **RPM from geometry** — converts steps/s ↔ RPM using full steps/rev and
  microstepping settings.
- **Live encoder feedback (closed loop)** — reads each SERVO42C magnetic encoder
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

| Motor | Drive            | EN      | STP     | DIR     |
| ----- | ---------------- | ------- | ------- | ------- |
| 1     | Direct (1:1)     | GPIO 17 | GPIO 27 | GPIO 22 |
| 2     | 5:1 planetary    | GPIO 2  | GPIO 3  | GPIO 4  |
| 3     | Direct (1:1)     | GPIO 10 | GPIO 9  | GPIO 11 |
| 4     | Direct (1:1)     | GPIO 0  | GPIO 5  | GPIO 6  |

`EN` is active-LOW on the SERVO42C; each motor's `GND` ties to the Pi `GND`.

### Encoder feedback (shared UART / TTL bus)

All drivers share one Pi UART in a multi-drop arrangement (Option A). Each driver
has a distinct address so only the addressed motor replies.

| Pi            | Pin             | SERVO42C (all)  |
| ------------- | --------------- | --------------- |
| TXD (GPIO 14) | physical pin 8  | Rx              |
| RXD (GPIO 15) | physical pin 10 | Tx              |
| GND           | —               | G               |

Leave each SERVO42C `3V3` pin unconnected.

SERVO42C UART defaults used here: **9600 baud**; addresses **`0xE0`–`0xE3`** for
motors 1–4 (OLED address slots `0`–`3`). Set the matching baud and address on each
driver's OLED menu.

A wiring diagram for the four-driver bus is in
[`docs/uart_bus_4_drivers.svg`](docs/uart_bus_4_drivers.svg).

> **Shared-bus tip:** put a small series resistor (~1 kΩ) on each driver's Tx to
> avoid contention on the shared Rx line, and make sure all drivers share a common
> ground with the Pi. A pull-up is optional — the SERVO42C idles its Tx HIGH.

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
| `SERVO_ADDR`  | `0xe0`         | Motor 1 SERVO42C address byte     |
| `SERVO_ADDR2` | `0xe1`         | Motor 2 SERVO42C address byte     |
| `SERVO_ADDR3` | `0xe2`         | Motor 3 SERVO42C address byte     |
| `SERVO_ADDR4` | `0xe3`         | Motor 4 SERVO42C address byte     |

## API

Routes are parameterized by motor id (`<mid>` = `1`–`4`).

| Method | Route                     | Description                                 |
| ------ | ------------------------- | ------------------------------------------- |
| GET    | `/`                       | Dashboard UI                                |
| POST   | `/motor/<mid>/enable`     | Energize and start the motor                |
| POST   | `/motor/<mid>/disable`    | Stop the motor (`{"soft": true\|false}`)   |
| POST   | `/motor/<mid>/direction`  | Set direction (`{"direction": "cw\|ccw"}`) |
| POST   | `/motor/<mid>/speed`      | Set speed in steps/s (1–6000)               |
| POST   | `/motor/<mid>/rpm`        | Set target RPM (output shaft)               |
| POST   | `/motor/<mid>/geometry`   | Set full steps/rev and microstepping        |
| POST   | `/motor/<mid>/accel`      | Set acceleration (steps/s²)                 |
| GET    | `/motor/<mid>/status`     | Motor status JSON                           |
| GET    | `/motor/<mid>/encoder`    | Encoder reading JSON                        |

### Encoder response

```json
{
  "available": true,
  "error": null,
  "carry": 148,
  "value": 11736,
  "counts": 9711064,
  "gear_ratio": 5.0,
  "angle_deg": 64.47,
  "total_angle_deg": 53344.47,
  "revolutions": 148.1791,
  "output_angle_deg": 12.89,
  "output_total_angle_deg": 10668.89,
  "output_revolutions": 29.6358
}
```

The encoder reports a 16-bit `value` (0–0xFFFF over one motor revolution) and a
signed `carry` that increments/decrements each full turn, so the total count
(`carry * 65536 + value`) tracks absolute motor position across many rotations.
The `output_*` fields divide motor-shaft motion by `gear_ratio` to give the geared
output-shaft position (identical to the motor shaft for the 1:1 motor).

## Project structure

```
led_app/
  app.py               Flask backend: motor control + encoder readers
  templates/
    index.html         Dashboard UI (per-motor controls + live encoder)
  speedtest.py         Measures actual motor speed via the encoder
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
