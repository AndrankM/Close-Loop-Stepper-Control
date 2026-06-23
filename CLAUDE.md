# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

A Raspberry Pi 5 Flask web app that drives a 4-axis robotic arm using NEMA 17 steppers and SERVO42C closed-loop drivers. The backend is a single large file (`led_app/app.py`, ~3500 lines). Development happens on Windows; the app runs on the Pi.

## Required packages

The app targets **Raspberry Pi OS** (Python 3.11+). `gpiozero`/`lgpio` must come from the system apt packages — building `lgpio` from pip requires `swig` and fails on the Pi. Create the venv with `--system-site-packages` so these are inherited.

**Core (always required):**
```bash
python3 -m venv --system-site-packages ~/led_app/venv
~/led_app/venv/bin/pip install flask pyserial
```

**Hardware PWM for axis 5/6 servos (optional, Pi 5 only):**
```bash
~/led_app/venv/bin/pip install rpi-hardware-pwm
```

**WS2812 emotion-ring LEDs (optional — requires SPI via spidev, not rpi_ws281x which is unsupported on Pi 5):**
```bash
sudo apt-get install python3-spidev
```

**Vision pipeline (optional — run the provided scripts on the Pi):**
```bash
scripts/setup_vision.sh   # apt installs python3-opencv + downloads YuNet + FER+ ONNX models to led_app/models/
scripts/setup_objects.sh  # downloads YOLOv4-tiny .weights/.cfg to led_app/models/
```

The app detects each dependency at import time and sets a `*_AVAILABLE` flag. Missing packages are logged but never crash the app.

## Running / deploying

**Run on the Pi (inside the venv):**
```bash
cd ~/led_app
./venv/bin/python app.py
# Dashboard at http://<pi-ip>:5000
```

**Deploy from Windows dev machine (copies `app.py` + `index.html`, restarts service):**
```powershell
./led_app/redeploy.ps1 -PiHost 192.168.0.103 -PiUser andpi5
```

**First-time Pi setup (run once on the Pi):**
```bash
cd ~/led_app
python3 -m venv --system-site-packages venv
./venv/bin/pip install flask pyserial
# Vision models (optional):
scripts/setup_vision.sh   # installs python3-opencv + YuNet/FER+ ONNX models
scripts/setup_objects.sh  # downloads YOLOv4-tiny model
```

The app runs gracefully without GPIO, serial, or CV2 (useful for local testing on non-Pi hardware — all hardware subsystems set an `_AVAILABLE` flag and degrade to no-ops).

## Architecture

### Single-file backend (`led_app/app.py`)

All backend logic lives in one file. It is divided into self-contained sections (marked with long comment banners):

- **ACT LED / SoC temp / Pi health** — reads `/proc`, `/sys`, and `vcgencmd` for the dashboard health card.
- **`StepperMotor` class** — one instance per physical motor (4 total, dict `motors`). Each motor runs a background `_run` thread that implements a trapezoidal velocity ramp by adjusting a `gpiozero.PWMOutputDevice` frequency. The `PWMOutputDevice` frequency equals the step rate in steps/s; `value=0.5` emits the pulse train, `value=0.0` stops it. End-stop inputs (shared or per-direction GPIO lines) are sampled in the same ramp loop to latch a `_blocked_dir`. Enable simply energizes and holds; `jog`/`run_pulses()` releases the `_pause` event to start motion.
- **`EncoderReader` class** — one instance per motor (dict `encoders`). Sends a 3-byte UART command to the addressed SERVO42C driver (address `0xE0`–`0xE3`) and parses the 8-byte reply (carry int32 + value uint16 + CRC). All four readers share one `serial.Serial` object and a single `_serial_lock`. Returns `counts = carry * 65536 + value` for absolute position tracking.
- **`TwinPoller`** — background thread that polls all four encoders at ~8 Hz and caches joint angles (radians) for the 3D digital-twin URDF viewer.
- **`RobotArm` class** — coordinates teach & playback. `capture()` snapshots encoder counts for all joints. `move_to_pose()` is the closed-loop proportional position controller: reads each encoder, computes error → `speed = KP * error`, runs until within `POS_TOLERANCE_COUNTS`. It also auto-learns per-joint wiring polarity (if the error grows it flips direction). `play()` sequences through waypoints in a background thread.
- **Servo control (axes 5 & 6)** — `rpi-hardware-pwm` drives GPIO 12/13 via RP1 PWM0 (auto-detected by hardware address `1f00098000`). Axis 5 has a slewing helper that moves in small steps to reduce shock.
- **`EmotionCamera` class** — one background thread: V4L2 capture at 640×480 → YuNet face detection → FER+ ONNX emotion classification (both via `cv2.dnn`) → publishes latest JPEG + result dict. Camera opens lazily and releases after 20 s idle. OpenCV threads are capped (`CV2_NUM_THREADS`) to leave CPU headroom for the software step-PWM.
- **`FaceTracker` class** — closed-loop face-following: reads the latest face box center from the camera, computes pixel error vs. frame center, mixes that error across motors 1–4 with configurable X/Y/rotation gains, and commands each motor's step rate proportionally. Axis 5 servo handles vertical tilt. All tuning constants are overridable via env vars (`FACE_TRACK_*`).
- **Flask routes** — defined at the bottom of `app.py`, parameterized by motor id (`<mid>` = 1–4). The dashboard SPA polls `/motor/<mid>/status`, `/motor/<mid>/encoder`, and `/system/health` on timers; action buttons POST to the REST endpoints.

### Frontend (`led_app/templates/index.html`)

A single-page app with no build step — plain HTML + inline JS (vanilla). Polls the REST API with `fetch`. The large `index.html` contains all motor cards, the teach/playback panel, the 3D twin viewer (Three.js + `URDFLoader`), and the Pi health card.

### Key global constants (tuning)

All in `app.py` near the top:

| Constant | Purpose |
|---|---|
| `POS_TOLERANCE_COUNTS` | Encoder stop band for closed-loop moves (~60 counts = 0.33°) |
| `POS_KP` | Proportional gain for closed-loop position (steps/s per count error) |
| `POS_SAFE_SPS` | Speed cap until move direction polarity is confirmed |
| `STEP_START_SPS` / `STEP_STOP_SPS` | Anti-resonance band (skip at low pulse rates) |
| `DEFAULT_ACCEL` | Ramp rate in steps/s² |
| `MOTOR{1-4}_GEAR_RATIO` | Motors 1 & 2 are 5:1 planetary; motors 3 & 4 are 1:1 |

### Environment variable overrides

| Variable | Default | Effect |
|---|---|---|
| `SERVO_UART` | `/dev/serial0` | Serial port for SERVO42C bus |
| `SERVO_BAUD` | `9600` | UART baud rate |
| `SERVO_ADDR`–`SERVO_ADDR4` | `0xe0`–`0xe3` | Per-motor SERVO42C addresses |
| `MIRO_CAMERA_DEVICE` | `/dev/video0` | USB camera path |
| `CV2_NUM_THREADS` | `1` | OpenCV thread cap (keep low to avoid step-PWM jitter) |
| `FACE_TRACK_*` | various | All face-tracker tuning params |
| `STEP_START_SPS` / `STEP_STOP_SPS` | `180` / `140` | Anti-resonance skip band |

### Hardware mapping quick reference

| Motor | EN | STP | DIR | Gear | End-stop |
|---|---|---|---|---|---|
| 1 | GPIO 17 | GPIO 27 | GPIO 22 | 5:1 | — |
| 2 | GPIO 2 | GPIO 3 | GPIO 4 | 5:1 | GPIO 5 (CW), GPIO 6 (CCW), active-low hall |
| 3 | GPIO 23 | GPIO 24 | GPIO 25 | 1:1 | GPIO 26 (shared, active-high) |
| 4 | GPIO 16 | GPIO 20 | GPIO 21 | 1:1 | GPIO 19 (shared, active-high) |

Axis 5 servo → GPIO 12 (HW PWM ch 0). Axis 6 servo → GPIO 13 (HW PWM ch 1).  
SERVO42C UART bus: Pi TXD GPIO 14 → all Rx; all Tx → Pi RXD GPIO 15. Baud 9600. Addresses 0xE0–0xE3.  
WS2812 emotion-ring LEDs → SPI0 MOSI GPIO 10 (pin 19); uses SPI bitbang because rpi_ws281x DMA is unsupported on Pi 5 RP1.

### Programs / teach data

Saved teach programs are JSON files in `led_app/programs/` (created at runtime, git-ignored). Each file is `{name}.json` and stores a list of waypoints, each with a `pose` dict (motor id → encoder counts) and optional `dwell`.
