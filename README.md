# Closed-Loop Stepper Control

A Raspberry Pi 5 web application for driving **four NEMA 17** stepper motors through
**MKS SERVO42C** drivers, with **live closed-loop feedback** from each driver's
built-in magnetic encoder over a shared UART bus.

The Flask backend controls the motors via GPIO step/direction signals and reads the
encoders over the serial port. A single-page dashboard provides per-motor motion
control plus a live encoder readout — angle dial, rotation counter, and a real-time
chart — for each motor.

Motors 3 and 4 are direct-drive; motors 1 and 2 each run through a **5:1 planetary
reducer**, and the app reports their RPM and rotation relative to the geared output
shaft.

## Features

- **Four independent motors** — each with its own control card and live encoder card.
- **Motor control** — enable/disable, direction (CW/CCW), speed (steps/s), and
  target RPM.
- **Hold-to-jog (jog-only motion)** — momentary CW/CCW jog buttons on each motor:
  the motor runs only while the button is held and ramps to a stop the instant it
  is released (works with mouse and touch, with pointer capture so it stops even if
  the pointer slides off). **Enable only energizes and holds** the joint in
  place — it never spins the motor on its own; all motion comes from jog or
  playback.
- **Trapezoidal acceleration / deceleration** — the live speed eases toward the
  target and back down to zero at a configurable acceleration (steps/s²), giving a
  symmetric trapezoidal velocity profile with smooth starts *and* stops.
- **End-stop / hall limit switches (Motors 2, 3 &amp; 4)** — motor 2 uses dedicated
  hall inputs per direction (**CW GPIO 5**, **CCW GPIO 6**, active-low), while
  motors 3 and 4 each use two hardware limit switches on a single shared GPIO line.
  Limits stop motion immediately in the blocked direction while still allowing jog
  away from the stop. A live end-stop indicator and an `END STOP` state appear on
  the motor card.
- **Teach &amp; playback** — record encoder poses and play them back as coordinated
  motion, Dobot-style (see [Teach &amp; playback](#teach--playback)).
- **Raspberry Pi 5 health monitoring** — a header temperature chip plus a
  collapsible health card showing CPU usage/load/frequency, SoC temperature,
  memory, disk, swap, uptime, network (IP + live throughput), active-cooler fan
  RPM, board power draw (V / A / W from the PMIC), and undervoltage/throttling
  flags (see [Pi 5 health](#raspberry-pi-5-health)).
- **Head camera + on-device emotion detection** — a USB camera (Innomaker
  U20CAM-1080P) mounted on the arm head streams a live annotated MJPEG feed. The
  Pi runs face detection (OpenCV **YuNet**) and emotion classification (**FER+**
  ONNX via `cv2.dnn`) fully on-device at ~15 fps. A collapsible **Camera** card on
  the main page shows the live feed, dominant-emotion verdict and per-emotion
  bars; the dedicated `/emotion` view shows the same at full size (and falls back
  to a browser simulation when the camera/model is unavailable).
- **Object detection** — an optional on-device **YOLOv4-tiny** detector (COCO 80
  classes) runs on its own thread and overlays labelled boxes on the same feed.
  Each detection reports its pixel box/centre **and a normalised offset from the
  frame centre (-1..1)** — the coordinate that will later drive the motors so the
  arm can physically track an object. Toggle it from the Camera card.
- **Global emergency stop** — a prominent header button plus a sticky floating
  button immediately hard-stop **all** motors and any running playback at once.
- **Hardware-timed step pulses** — step signals are generated with hardware PWM
  (frequency = step rate), giving accurate, linear speed control up to 6000 steps/s.
- **Acceleration ramp** — smooth ramp toward the target speed with a configurable
  acceleration (steps/s²).
- **Gear-ratio aware** — per-motor gear ratios (e.g. motors 1 and 2's 5:1 planetary
  reducers) are applied so RPM and encoder turns/angle are reported at the
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
| 1     | 5:1 planetary    | GPIO 17 | GPIO 27 | GPIO 22 |
| 2     | 5:1 planetary    | GPIO 2  | GPIO 3  | GPIO 4  |
| 3     | Direct (1:1)     | GPIO 23 | GPIO 24 | GPIO 25 |
| 4     | Direct (1:1)     | GPIO 16 | GPIO 20 | GPIO 21 |

`EN` is active-LOW on the SERVO42C; each motor's `GND` ties to the Pi `GND`.

### End-stop / hall limit switches (Motors 2, 3 &amp; 4)

Motor 2 uses two dedicated **hall-effect** limit inputs, one per direction:

| Motor | Sensor type | Direction | GPIO   | Idle | Triggered |
| ----- | ----------- | --------- | ------ | ---- | --------- |
| 2     | Hall        | CW        | GPIO 5 | HIGH | LOW (0 V) |
| 2     | Hall        | CCW       | GPIO 6 | HIGH | LOW (0 V) |

These are configured as active-low in software (`pull_up=True`), matching hall
outputs that pull the pin to 0 V at limit.

Motors 3 and 4 each keep the shared-line end-stop layout:

Motors 3 and 4 each have two travel-limit switches sharing a **single GPIO line**
(GPIO 26 for motor 3, GPIO 19 for motor 4). Each switch is wired to **3.3 V** (the
Pi GPIO is 3.3 V tolerant only — **never wire a GPIO to 5 V**) so a pressed switch
drives the pin HIGH; an internal pull-down holds it LOW when released.

| Motor | Shared travel limit | Idle | Pressed |
| ----- | ------------------- | ---- | ------- |
| 3     | GPIO 26             | LOW  | HIGH    |
| 4     | GPIO 19             | LOW  | HIGH    |

For motors 3/4, only one end stop can be reached at a time, so the motor's
**current travel direction** identifies which limit was hit — no separate pin per
switch is needed. When the line trips, the motor stops immediately and that
direction stays blocked until the switch releases; jogging the opposite direction
backs it off.

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
| POST   | `/motor/<mid>/enable`     | Energize and hold position (does not move)  |
| POST   | `/motor/<mid>/disable`    | Stop the motor (`{"soft": true\|false}`)   |
| POST   | `/motor/<mid>/jog`        | Momentary jog (`{"direction":"cw\|ccw","on":true\|false}`) |
| POST   | `/motor/<mid>/direction`  | Set direction (`{"direction": "cw\|ccw"}`) |
| POST   | `/motor/<mid>/speed`      | Set speed in steps/s (1–6000)               |
| POST   | `/motor/<mid>/rpm`        | Set target RPM (output shaft)               |
| POST   | `/motor/<mid>/geometry`   | Set full steps/rev and microstepping        |
| POST   | `/motor/<mid>/accel`      | Set acceleration (steps/s²)                 |
| GET    | `/motor/<mid>/status`     | Motor status JSON (incl. limit switch state)|
| GET    | `/motor/<mid>/encoder`    | Encoder reading JSON                        |
| POST   | `/estop`                  | Emergency stop: hard-stop all motors + playback |
| GET    | `/emotion`                | Emotion detection page                      |
| GET    | `/emotion/latest`         | Latest face/emotion result JSON (`live`, `box`, `scores`, `dominant`, `fps`) |
| GET    | `/camera/stream`          | Annotated live MJPEG stream (`multipart/x-mixed-replace`) |
| GET    | `/camera/snapshot`        | Single annotated JPEG frame                 |
| GET    | `/objects/latest`         | Latest object detections (boxes, centres, normalised offsets, primary) |
| POST   | `/objects/enable`         | Enable/disable object detection (`{"on": true\|false}`) |
| GET    | `/emotion/rings`          | Emotion rings status JSON (`enabled`, `available`, backend info) |
| POST   | `/emotion/rings`          | Enable/disable emotion rings (`{"on": true\|false}`) |
| POST   | `/emotion/rings/test`     | Run LED ring self-test sweep |
| POST   | `/led/on`                 | Turn Pi ACT LED on |
| POST   | `/led/off`                | Turn Pi ACT LED off |
| GET    | `/led/status`             | Pi ACT LED status JSON |
| GET    | `/system/temp`            | Pi 5 SoC temperature JSON                   |
| GET    | `/system/health`          | Pi 5 health JSON (CPU, memory, disk, swap, network, fan, power, throttling) |

### Teach &amp; playback API

| Method | Route                | Description                                          |
| ------ | -------------------- | ---------------------------------------------------- |
| POST   | `/arm/capture`       | Snapshot all joints' encoder positions (a waypoint)  |
| POST   | `/arm/freedrive`     | `{"on": true\|false}` de-energize to hand-guide / re-hold |
| POST   | `/arm/play`          | Play waypoints `{waypoints, speed, loops, dwell}`    |
| POST   | `/arm/stop`          | Stop playback                                        |
| GET    | `/arm/status`        | Playback state JSON                                  |
| GET    | `/programs`          | List saved programs                                  |
| GET    | `/programs/<name>`   | Load a saved program                                 |
| POST   | `/programs/<name>`   | Save a program (JSON body)                           |
| DELETE | `/programs/<name>`   | Delete a program                                     |

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
output-shaft position (identical to the motor shaft for the 1:1 motors).

## Teach &amp; playback

The dashboard includes a **Teach &amp; Playback** panel for recording arm poses and
replaying them as coordinated motion, similar to Dobot Studio.

- **Teach** a pose in two ways:
  - **Free-drive** \u2014 toggle free-drive to de-energize all motors so the arm can be
    moved by hand, position it, then **Record Waypoint** to capture every joint's
    encoder position.
  - **Jog** \u2014 use the per-motor hold-to-jog (or any motor control) to position the
    arm, then **Record Waypoint**.
- **Playback** drives **all joints simultaneously** toward each waypoint using
  software closed-loop position control (read encoder \u2192 proportional speed toward
  target \u2192 stop within a tolerance band). Each joint auto-learns its wiring polarity
  and is speed-limited until its move direction is confirmed.
- **Options** \u2014 adjustable playback speed, per-waypoint dwell time, and loop/repeat
  N times (0 = infinite).
- **Programs** \u2014 save named programs as JSON on the Pi (under `led_app/programs/`)
  and load or delete them later. Program names are validated to keep them within
  that folder.

Closed-loop tuning constants live in `app.py`: `POS_TOLERANCE_COUNTS` (stop band),
`POS_KP` (proportional gain), `POS_TIMEOUT_S`, `POS_APPROACH_MIN_SPS`, and
`POS_SAFE_SPS` (speed cap until direction is confirmed).

## Raspberry Pi 5 health

The dashboard header shows a live **temperature chip**; click it to expand a
collapsible **health card** that polls `/system/health` every few seconds and shows:

- **CPU** — utilisation %, 1-minute load average, and current core frequency
- **Temperature** — SoC temperature with a bar scaled to the 85 °C limit
- **Memory** and **Swap** — used / total with usage bars
- **Disk (/)** — used / total with a usage bar
- **Network** — primary IPv4 address, active interface, and live RX/TX throughput
- **Fan** — Raspberry Pi 5 active-cooler tachometer (RPM)
- **Power draw** — total board power in watts, derived from the PMIC per-rail
  voltages and currents (`vcgencmd pmic_read_adc`), with the 5 V input voltage and
  estimated input current
- **Power &amp; throttling** — undervoltage / frequency-cap / throttling / soft
  temperature-limit flags (`vcgencmd get_throttled`), highlighted red when active

These readings use only the Python standard library plus `vcgencmd`, and each metric
degrades gracefully (`n/a`) on hardware that does not expose it.

## Head camera &amp; emotion detection

A USB camera (**Innomaker U20CAM-1080P**, a standard UVC/V4L2 device on
`/dev/video0`) mounted on the arm head feeds an on-device vision pipeline:

- **Capture** — OpenCV `VideoCapture` (V4L2, MJPG) at 640×480.
- **Face detection** — OpenCV **YuNet** (`face_detection_yunet_2023mar.onnx`), with
  the bundled Haar cascade as an automatic fallback.
- **Emotion** — Microsoft **FER+** (`emotion-ferplus-8.onnx`) run through
  `cv2.dnn` (no extra runtime needed). The 8 FER+ classes are mapped onto the six
  shown emotions (happy, neutral, surprise, sad, angry, fear).
- **Object detection** — optional **YOLOv4-tiny** (`yolov4-tiny.weights` + `.cfg`,
  COCO 80 classes) via `cv2.dnn.DetectionModel`, on a separate thread so it does
  not slow the emotion/stream loop. Returns, per object, the pixel box and centre
  plus a normalised `offset` (`-1..1`) from the frame centre — the value to feed a
  pan/tilt (motor) controller for physical tracking. `primary` is the
  highest-confidence object.

A single background thread grabs frames, detects the largest face, classifies its
emotion, draws the box + label, and publishes the latest annotated JPEG plus a
structured result. The camera opens lazily on first request and releases itself
after ~20 s with no viewers. Runs at roughly 15 fps with ~35 ms inference per
frame on the Pi 5.

Open the **Camera** card on the main page (or the full `/emotion` view) to watch
the live `/camera/stream` with the dominant emotion and per-emotion bars driven by
`/emotion/latest`.

### Setup (one-time, on the Pi)

OpenCV and the model files are not part of the repo. Install and download them with:

```bash
# from the project root, with the Pi reachable over SSH
scripts/setup_vision.sh        # run on the Pi (installs python3-opencv + face/emotion models)
scripts/setup_objects.sh       # run on the Pi (downloads the YOLOv4-tiny object model)
```

This installs the `python3-opencv` apt package and downloads the YuNet, FER+ and
YOLOv4-tiny models into `led_app/models/` (git-ignored). Set `MIRO_CAMERA_DEVICE`
to override the default `/dev/video0`.

## Project structure

```
led_app/
  app.py               Flask backend: motor control + encoder readers + arm + Pi health + camera
  templates/
    index.html         Dashboard UI (per-motor controls + live encoder + teach/playback + Pi health + camera)
    emotion.html       Emotion detection page (live feed, emotion bars)
  models/              YuNet + FER+ ONNX models (downloaded by scripts/setup_vision.sh, git-ignored)
  programs/            Saved teach/playback programs (JSON, created at runtime)
  speedtest.py         Measures actual motor speed via the encoder
  redeploy.ps1         Deploy script (scp + restart service over SSH)
blink_led.py           Standalone onboard LED blink example
```

## Deployment

`led_app/redeploy.ps1` copies `app.py` and `index.html` to the Pi and restarts the
service:

```powershell
./led_app/redeploy.ps1 -PiHost 192.168.0.101 -PiUser andpi5
```

## License

MIT
