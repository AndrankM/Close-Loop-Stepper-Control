import os
import json
import math
import re
import socket
import threading
import time
import shutil
import subprocess

from flask import Flask, jsonify, render_template, request, Response

try:
    from gpiozero import (
        DigitalOutputDevice,
        PWMOutputDevice,
        DigitalInputDevice,
    )

    GPIO_AVAILABLE = True
except Exception:  # gpiozero not installed / not running on a Pi
    DigitalOutputDevice = None
    PWMOutputDevice = None
    DigitalInputDevice = None
    GPIO_AVAILABLE = False

try:
    # True hardware PWM on the Pi 5 RP1 chip. Unlike pigpio (which does not
    # support the Pi 5) or gpiozero software PWM, this drives GPIO 12/13 with a
    # jitter-free kernel-timed pulse train, eliminating servo bouncing.
    from rpi_hardware_pwm import HardwarePWM

    HW_PWM_AVAILABLE = True
except Exception:  # rpi-hardware-pwm not installed
    HardwarePWM = None
    HW_PWM_AVAILABLE = False

try:
    import serial  # pyserial

    SERIAL_AVAILABLE = True
except Exception:  # pyserial not installed
    serial = None
    SERIAL_AVAILABLE = False

try:
    import cv2
    import numpy as np

    CV2_AVAILABLE = True
    # The stepper STEP pulses are produced by software-timed PWM (gpiozero on
    # ordinary GPIO). OpenCV's DNN inference otherwise fans out across EVERY CPU
    # core and starves that PWM thread, which shows up as motor vibration the
    # moment the camera/emotion detection turns on. Cap OpenCV to a subset of
    # cores so at least one core stays free to clock the step pulses cleanly.
    try:
        cv2.setNumThreads(int(os.environ.get("CV2_NUM_THREADS", "1")))
    except Exception:
        pass
except Exception:  # opencv not installed / headless dev box
    cv2 = None
    np = None
    CV2_AVAILABLE = False

try:
    from rpi_ws281x import PixelStrip, Color

    WS281X_AVAILABLE = True
except Exception:  # ws281x library missing / not running on a Pi
    PixelStrip = None
    Color = None
    WS281X_AVAILABLE = False

try:
    import pigpio

    PIGPIO_AVAILABLE = True
except Exception:  # pigpio module missing
    pigpio = None
    PIGPIO_AVAILABLE = False

try:
    import spidev

    SPIDEV_AVAILABLE = True
except Exception:  # spidev module missing
    spidev = None
    SPIDEV_AVAILABLE = False

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Onboard ACT LED control
# ---------------------------------------------------------------------------
LED_TRIGGER = "/sys/class/leds/ACT/trigger"
LED_BRIGHTNESS = "/sys/class/leds/ACT/brightness"


def write_led(path, value):
    with open(path, "w") as f:
        f.write(value)


def read_brightness():
    with open(LED_BRIGHTNESS, "r") as f:
        return f.read().strip()


# ---------------------------------------------------------------------------
# Onboard SoC temperature
# ---------------------------------------------------------------------------
CPU_TEMP_PATH = "/sys/class/thermal/thermal_zone0/temp"


def read_cpu_temp_c():
    """Return the Pi SoC temperature in degrees Celsius, or None if unavailable."""
    try:
        with open(CPU_TEMP_PATH, "r") as f:
            milli = int(f.read().strip())
        return round(milli / 1000.0, 1)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Onboard health (CPU load, memory, disk, uptime, throttling, voltage)
# ---------------------------------------------------------------------------
# Cached snapshot of /proc/stat so CPU usage can be measured as a delta.
_cpu_stat_prev = {"total": 0, "idle": 0}

# Flag bits returned by `vcgencmd get_throttled`. The low bits mean the
# condition is happening right now; the high bits mean it occurred since boot.
_THROTTLE_FLAGS = {
    0: "under-voltage",
    1: "arm frequency capped",
    2: "currently throttled",
    3: "soft temperature limit",
}


def _read_cpu_percent():
    """CPU utilisation since the previous call, as a 0-100 percentage."""
    try:
        with open("/proc/stat", "r") as f:
            parts = f.readline().split()
        vals = [int(v) for v in parts[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)  # idle + iowait
        total = sum(vals)
        d_total = total - _cpu_stat_prev["total"]
        d_idle = idle - _cpu_stat_prev["idle"]
        _cpu_stat_prev["total"] = total
        _cpu_stat_prev["idle"] = idle
        if d_total <= 0:
            return None
        return round(100.0 * (d_total - d_idle) / d_total, 1)
    except Exception:
        return None


def _read_meminfo():
    """Return (used_mb, total_mb, percent) from /proc/meminfo."""
    try:
        info = {}
        with open("/proc/meminfo", "r") as f:
            for line in f:
                key, _, rest = line.partition(":")
                info[key] = int(rest.strip().split()[0])  # kB
        total = info.get("MemTotal", 0)
        avail = info.get("MemAvailable", info.get("MemFree", 0))
        used = total - avail
        pct = round(100.0 * used / total, 1) if total else None
        return round(used / 1024.0), round(total / 1024.0), pct
    except Exception:
        return None, None, None


def _read_uptime():
    """System uptime in seconds, or None."""
    try:
        with open("/proc/uptime", "r") as f:
            return int(float(f.read().split()[0]))
    except Exception:
        return None


def _read_cpu_freq_mhz():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r") as f:
            return round(int(f.read().strip()) / 1000.0)  # kHz -> MHz
    except Exception:
        return None


def _vcgencmd(*args):
    try:
        out = subprocess.run(
            ["vcgencmd", *args], capture_output=True, text=True, timeout=2.0
        )
        return out.stdout.strip()
    except Exception:
        return None


def _read_voltage():
    """Core voltage in volts, e.g. 'volt=0.8800V' -> 0.88."""
    raw = _vcgencmd("measure_volts")
    if not raw:
        return None
    m = re.search(r"([\d.]+)", raw)
    return round(float(m.group(1)), 3) if m else None


def _read_power():
    """Total board power from the Pi 5 PMIC.

    `vcgencmd pmic_read_adc` reports a current and a voltage for each supply
    rail. Summing volts*amps over every rail gives the board's real power draw;
    the input current is estimated from that power and the 5V supply rail.
    Returns {'voltage_v', 'current_a', 'power_w'} or None on a non-PMIC board.
    """
    raw = _vcgencmd("pmic_read_adc")
    if not raw:
        return None
    amps, volts = {}, {}
    supply_v = None
    for line in raw.splitlines():
        m = re.match(r"\s*(\S+?)_([AV])\s+\w+\(\d+\)=([\d.]+)", line)
        if not m:
            continue
        rail, kind, val = m.group(1), m.group(2), float(m.group(3))
        if kind == "A":
            amps[rail] = val
        else:
            volts[rail] = val
            if rail == "EXT5V":
                supply_v = val
    if not amps:
        return None
    power_w = sum(volts[r] * amps[r] for r in amps if r in volts)
    if not supply_v:
        supply_v = volts.get("EXT5V") or 5.0
    current_a = power_w / supply_v if supply_v else None
    return {
        "voltage_v": round(supply_v, 2),
        "current_a": round(current_a, 2) if current_a is not None else None,
        "power_w": round(power_w, 2),
    }


def _read_throttled():
    """Parse `vcgencmd get_throttled` into active/past condition lists."""
    raw = _vcgencmd("get_throttled")  # e.g. 'throttled=0x50005'
    if not raw or "=" not in raw:
        return None
    try:
        bits = int(raw.split("=")[1], 16)
    except ValueError:
        return None
    active, past = [], []
    for bit, name in _THROTTLE_FLAGS.items():
        if bits & (1 << bit):
            active.append(name)
        if bits & (1 << (bit + 16)):
            past.append(name)
    return {"raw": raw.split("=")[1], "active": active, "past": past, "ok": not active}


def _read_swap():
    """Return (used_mb, total_mb, percent) of swap from /proc/meminfo."""
    try:
        info = {}
        with open("/proc/meminfo", "r") as f:
            for line in f:
                key, _, rest = line.partition(":")
                if key in ("SwapTotal", "SwapFree"):
                    info[key] = int(rest.strip().split()[0])  # kB
        total = info.get("SwapTotal", 0)
        free = info.get("SwapFree", 0)
        used = total - free
        pct = round(100.0 * used / total, 1) if total else 0.0
        return round(used / 1024.0), round(total / 1024.0), pct
    except Exception:
        return None, None, None


# Cached /proc/net/dev counters + timestamp so throughput is a per-second rate.
_net_prev = {"t": 0.0, "rx": 0, "tx": 0}


def _read_network():
    """Primary IPv4 address plus RX/TX throughput in kB/s since the last call."""
    result = {"ip": None, "iface": None, "rx_kbps": None, "tx_kbps": None}
    # Best-effort source IP via a dummy UDP socket (no packets are sent).
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            result["ip"] = s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        pass
    try:
        rx_total = tx_total = 0
        iface = None
        with open("/proc/net/dev", "r") as f:
            for line in f:
                if ":" not in line:
                    continue
                name, _, data = line.partition(":")
                name = name.strip()
                if name == "lo":
                    continue
                cols = data.split()
                rx_total += int(cols[0])
                tx_total += int(cols[8])
                if iface is None and int(cols[0]) > 0:
                    iface = name
        result["iface"] = iface
        now = time.time()
        dt = now - _net_prev["t"]
        if _net_prev["t"] and dt > 0:
            result["rx_kbps"] = round((rx_total - _net_prev["rx"]) / dt / 1024.0, 1)
            result["tx_kbps"] = round((tx_total - _net_prev["tx"]) / dt / 1024.0, 1)
        _net_prev.update({"t": now, "rx": rx_total, "tx": tx_total})
    except Exception:
        pass
    return result


def _read_fan_rpm():
    """Pi 5 active-cooler tachometer (fan1_input under any hwmon node), or None."""
    try:
        import glob
        for path in glob.glob("/sys/class/hwmon/hwmon*/fan1_input"):
            with open(path, "r") as f:
                return int(f.read().strip())
    except Exception:
        pass
    return None


def read_health():
    """Aggregate Pi health metrics into one dict for the UI."""
    used_mb, total_mb, mem_pct = _read_meminfo()
    try:
        du = shutil.disk_usage("/")
        disk = {
            "used_gb": round(du.used / 1e9, 1),
            "total_gb": round(du.total / 1e9, 1),
            "percent": round(100.0 * du.used / du.total, 1),
        }
    except Exception:
        disk = None
    try:
        load1 = round(os.getloadavg()[0], 2)
    except Exception:
        load1 = None
    swap_used, swap_total, swap_pct = _read_swap()
    return {
        "temp_c": read_cpu_temp_c(),
        "cpu_percent": _read_cpu_percent(),
        "load1": load1,
        "cpu_freq_mhz": _read_cpu_freq_mhz(),
        "mem": {"used_mb": used_mb, "total_mb": total_mb, "percent": mem_pct},
        "swap": {"used_mb": swap_used, "total_mb": swap_total, "percent": swap_pct},
        "disk": disk,
        "uptime_s": _read_uptime(),
        "voltage_v": _read_voltage(),
        "throttled": _read_throttled(),
        "net": _read_network(),
        "fan_rpm": _read_fan_rpm(),
        "power": _read_power(),
    }


# ---------------------------------------------------------------------------
# NEMA 17 + MKS SERVO42C stepper control
#   Motor 1:  EN -> GPIO 17   STP -> GPIO 27   DIR -> GPIO 22
#   Motor 2:  EN -> GPIO 2    STP -> GPIO 3    DIR -> GPIO 4
#   Motor 3:  EN -> GPIO 23   STP -> GPIO 24   DIR -> GPIO 25
#   Motor 4:  EN -> GPIO 16   STP -> GPIO 20   DIR -> GPIO 21
#   (EN is active-LOW on the SERVO42C)
#   NOTE: Motor 3 was moved off GPIO 9/10/11 because those are the SPI0 pins
#   (GPIO10 = MOSI) used to drive the WS2812 emotion rings on the Pi 5.
# ---------------------------------------------------------------------------
EN_PIN = 17
STP_PIN = 27
DIR_PIN = 22

EN2_PIN = 2
STP2_PIN = 3
DIR2_PIN = 4

EN3_PIN = 23
STP3_PIN = 24
DIR3_PIN = 25

EN4_PIN = 16
STP4_PIN = 20
DIR4_PIN = 21

# Gearbox reduction (motor revs : output revs). Motors 1 and 2 each run through a
# 5:1 planetary reducer, so their output shafts turn 5x slower than the motor
# shaft (and the encoder, which sits on the motor shaft). Motors 3 and 4 default
# to direct-drive (1:1); change if they have reducers.
MOTOR1_GEAR_RATIO = 5.0
MOTOR2_GEAR_RATIO = 5.0
MOTOR3_GEAR_RATIO = 1.0
MOTOR4_GEAR_RATIO = 1.0

# Motor 2 dedicated Hall end-stops:
#   CW  limit -> GPIO 5
#   CCW limit -> GPIO 6
# Sensors are active-LOW: idle = 3.3 V, triggered (at limit) = 0 V.
# pull_up=True (internal pull-up) → is_active when LOW.
# Override per sensor via env var if wiring differs:
#   M2_LIMIT_CW_ACTIVE_LOW=0  or  M2_LIMIT_CCW_ACTIVE_LOW=0
M2_LIMIT_CW_PIN = 5
M2_LIMIT_CCW_PIN = 6
M2_LIMIT_ACTIVE_LOW = True  # active-LOW sensors: is_active when pin = 0 V
def _env_bool(name, default):
    v = os.environ.get(name, "").strip().lower()
    return default if v == "" else v not in ("0", "false", "no", "off")
M2_LIMIT_CW_ACTIVE_LOW  = _env_bool("M2_LIMIT_CW_ACTIVE_LOW",  M2_LIMIT_ACTIVE_LOW)
M2_LIMIT_CCW_ACTIVE_LOW = _env_bool("M2_LIMIT_CCW_ACTIVE_LOW", M2_LIMIT_ACTIVE_LOW)

# Motor 3 end-stop limit switches. Both travel-limit switches share a SINGLE
# GPIO line (GPIO 26): each is wired to 3.3V (the Pi GPIO is 3.3V only — never
# 5V), so a pressed switch drives the pin HIGH and an internal pull-down holds
# it LOW when released. Only one end stop can be reached at a time, so the
# motor's current travel direction tells us which limit was hit — no need for a
# separate pin per switch. When the line trips the motor stops immediately and
# refuses to drive further that way; jogging the opposite direction backs off.
M3_LIMIT_PIN = 26
M4_LIMIT_PIN = 19

# Servo motors for axis 5 & 6 (DX-227 270-degree servos via hardware PWM).
# GPIO 12/13 are real hardware PWM pins on the Pi 5 (RP1 pwmchip2). Hardware
# PWM produces a perfectly stable pulse train, which removes the idle jitter
# seen with gpiozero software PWM (pigpio does not support the Pi 5 at all).
SERVO5_PIN = 12  # physical pin 32 -> PWM channel 0
SERVO6_PIN = 13  # physical pin 33 -> PWM channel 1
SERVO5_CHANNEL = 0
SERVO6_CHANNEL = 1
# RP1 PWM0 (hardware address 1f00098000) drives GPIO 12/13 on the Pi 5. The
# sysfs pwmchip index for this block is not fixed, so it is auto-detected at
# runtime; this value is only a fallback if detection fails.
SERVO_PWM_CHIP = 0
SERVO_PWM_HW_ADDR = "1f00098000"  # RP1 PWM0 peripheral address
SERVO_PWM_HZ = 50  # 20 ms frame
SERVO_PERIOD_US = 20000
SERVO_MIN_ANGLE = -135
SERVO_MAX_ANGLE = 135
# DX-227 pulse range (microseconds): 500us = -135°, 1500us = 0°, 2500us = +135°
SERVO_MIN_PULSE_US = 500
SERVO_CENTER_PULSE_US = 1500
SERVO_MAX_PULSE_US = 2500

# Driver enable pin is active-LOW: drive LOW to energize the coils.
EN_ACTIVE_LOW = True

# Speed limits in steps per second. The hardware-timed PWM step generator can
# drive well past the old 2000 cap; 6000 sps = ~112 motor RPM at 1/16 stepping.
MIN_SPEED = 1
MAX_SPEED = 6000
DEFAULT_SPEED = 400

# Anti-resonance speed band (steps/sec). Many NEMA17 systems can bounce or
# chatter at very low pulse rates; skip that band on start and cut motion just
# above it on stop so all axes feel smooth.
STEP_START_SPS = int(os.environ.get("STEP_START_SPS", "180"))
STEP_STOP_SPS = int(os.environ.get("STEP_STOP_SPS", "140"))

# Mechanical / driver geometry used to convert between steps/sec and RPM.
#   full_steps_per_rev: motor's native step count (1.8 deg NEMA 17 -> 200)
#   microstepping:      the SERVO42C microstep setting (e.g. 1, 16, 32, 256)
DEFAULT_FULL_STEPS_PER_REV = 200
DEFAULT_MICROSTEPPING = 16

# Acceleration ramp in steps per second^2. The running speed eases toward the
# target instead of jumping, which avoids stalling/skipping at high speeds.
MIN_ACCEL = 100
MAX_ACCEL = 50000
DEFAULT_ACCEL = 2500

# ---------------------------------------------------------------------------
# Closed-loop positioning (teach & playback)
#   Encoder is 65536 counts / motor revolution. The controller drives each
#   joint toward a recorded encoder count and stops within a tolerance band.
# ---------------------------------------------------------------------------
POS_TOLERANCE_COUNTS = 60       # ~0.33 deg at 65536 counts/rev
POS_TIMEOUT_S = 20.0            # max time to reach one waypoint per joint
POS_APPROACH_MIN_SPS = 40      # floor speed so the joint keeps creeping in
POS_KP = 0.6                   # proportional gain: steps/sec per count error
POS_SAFE_SPS = 400             # speed cap until move direction is confirmed
PROGRAMS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "programs"
)


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _as_bool(value, default=False):
    """Parse booleans robustly from JSON/strings."""
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        s = value.strip().lower()
        if s in ("1", "true", "yes", "on"):
            return True
        if s in ("0", "false", "no", "off"):
            return False
    return bool(value)


class StepperMotor:
    """Generates step pulses on a background thread while the motor is enabled."""

    def __init__(self, en_pin, stp_pin, dir_pin, gear_ratio=1.0,
                 limit_pin=None, limit_pin_cw=None, limit_pin_ccw=None,
                 limit_active_low=False,
                 limit_cw_active_low=None, limit_ccw_active_low=None):
        self._lock = threading.Lock()
        self.enabled = False
        self.stopping = False
        self.direction = "cw"  # "cw" or "ccw"
        self.speed = DEFAULT_SPEED  # target steps per second
        self.current_speed = 0.0  # live (ramped) steps per second
        self.accel = DEFAULT_ACCEL  # steps per second^2
        self.full_steps_per_rev = DEFAULT_FULL_STEPS_PER_REV
        self.microstepping = DEFAULT_MICROSTEPPING
        self.gear_ratio = float(gear_ratio) or 1.0  # motor revs : output revs
        # End-stop state. Supports either a single shared line (legacy) or
        # dedicated CW/CCW inputs.
        self.limit_stop = False
        self._blocked_dir = None
        self._limit_active_low = bool(limit_active_low)
        # Per-direction polarity: allow CW and CCW sensors to be wired differently.
        # Falls back to limit_active_low if not specified.
        _cw_al  = limit_active_low if limit_cw_active_low  is None else limit_cw_active_low
        _ccw_al = limit_active_low if limit_ccw_active_low is None else limit_ccw_active_low
        self._stop = threading.Event()
        self._soft_stop = threading.Event()
        # When set, the worker keeps the coils energized but emits no pulses
        # (used to HOLD a position during closed-loop moves / playback).
        self._pause = threading.Event()
        self._thread = None
        # Live step-PWM output state. The pulse train is only (re)written when
        # one of these actually changes; re-issuing the frequency/value every
        # ramp cycle restarts the pulse train on the Pi PWM backend, which
        # corrupts step timing and makes the motor vibrate/bounce.
        self._pwm_on = False
        self._last_freq = 0

        if GPIO_AVAILABLE:
            # The device handles active-low inversion so "off" leaves the
            # driver disabled (EN pin held HIGH).
            self._en = DigitalOutputDevice(
                en_pin, active_high=not EN_ACTIVE_LOW, initial_value=False
            )
            # Step pulses are produced by a hardware-timed PWM square wave
            # (frequency = step rate, 50% duty). This keeps pulse timing
            # accurate and linear at high speeds, which a Python sleep-loop
            # cannot do. value=0 means "no pulses".
            self._step = PWMOutputDevice(
                stp_pin, frequency=max(1, int(DEFAULT_SPEED)), initial_value=0.0
            )
            self._dir = DigitalOutputDevice(dir_pin, initial_value=False)
            # Optional end-stop inputs. pull_up=False means active-HIGH input;
            # pull_up=True means active-LOW input (idle HIGH, trigger LOW).
            self._limit = (
                DigitalInputDevice(
                    limit_pin,
                    pull_up=self._limit_active_low,
                    bounce_time=0.005,
                )
                if limit_pin is not None else None
            )
            self._limit_cw = (
                DigitalInputDevice(
                    limit_pin_cw,
                    pull_up=bool(_cw_al),
                    bounce_time=0.005,
                )
                if limit_pin_cw is not None else None
            )
            self._limit_ccw = (
                DigitalInputDevice(
                    limit_pin_ccw,
                    pull_up=bool(_ccw_al),
                    bounce_time=0.005,
                )
                if limit_pin_ccw is not None else None
            )
        else:
            self._en = self._step = self._dir = None
            self._limit = None
            self._limit_cw = None
            self._limit_ccw = None

    # -- worker -------------------------------------------------------------
    def _run(self):
        # The worker only ramps current_speed toward the target and updates the
        # PWM frequency; the hardware PWM does the precise pulse timing. The
        # ramp loop's own timing can be loose without affecting motor speed.
        RAMP_DT = 0.02  # seconds between ramp updates
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now

            # The target is the running speed while actively jogging/driving,
            # or zero when held (jog released) or soft-stopping. current_speed
            # always eases toward the target at the configured accel rate, so
            # BOTH the ramp-up and the ramp-down are smooth — a trapezoidal
            # velocity profile rather than an abrupt start/stop.
            held = self._pause.is_set()
            soft = self._soft_stop.is_set()

            # End-stop behavior:
            # - Dedicated CW/CCW lines: block only the matching direction.
            # - Shared line: latch currently moving direction until release.
            if self._limit_cw is not None or self._limit_ccw is not None:
                pressed_cw = self._limit_pressed_cw()
                pressed_ccw = self._limit_pressed_ccw()
                if pressed_cw and pressed_ccw:
                    # CW and CCW limits cannot both be active simultaneously —
                    # treat as sensor noise / wiring fault and ignore both so
                    # the motor is never locked by a spurious double-read.
                    pressed_cw = False
                    pressed_ccw = False
                if pressed_cw:
                    self._blocked_dir = "cw"
                elif pressed_ccw:
                    self._blocked_dir = "ccw"
                else:
                    self._blocked_dir = None
                limited = (
                    (pressed_cw and self.direction == "cw")
                    or (pressed_ccw and self.direction == "ccw")
                )
            else:
                pressed = self._limit_pressed()
                if pressed:
                    if self._blocked_dir is None and self.current_speed > 0 \
                            and not held and not soft:
                        self._blocked_dir = self.direction
                else:
                    self._blocked_dir = None
                limited = pressed and self._blocked_dir == self.direction
            self.limit_stop = limited
            if limited:
                self.current_speed = 0.0

            target = 0.0 if (held or soft or limited) else float(self.speed)

            step_accel = self.accel * dt
            if self.current_speed < target:
                self.current_speed = min(target, self.current_speed + step_accel)
                # Skip the low-speed resonance band on ramp-up: jump from below
                # STEP_STOP_SPS directly to STEP_START_SPS rather than crawling
                # through it and causing vibration/chatter.
                if STEP_STOP_SPS < self.current_speed < STEP_START_SPS:
                    self.current_speed = min(target, float(STEP_START_SPS))
            elif self.current_speed > target:
                self.current_speed = max(0.0, self.current_speed - step_accel)
                # Cut to zero below STEP_STOP_SPS on a ramp-to-stop so the motor
                # never lingers in the resonance band on deceleration.
                if target <= 0.0 and 0.0 < self.current_speed <= STEP_STOP_SPS:
                    self.current_speed = 0.0

            # A soft-stop fully ramps down, then exits the worker (de-energize).
            if soft and self.current_speed <= 0.0:
                break

            if self.current_speed <= 0.0:
                # Fully stopped: keep the coils energized but emit no pulses
                # (position hold). The smooth ramp-down already happened above.
                if self._step is not None and self._pwm_on:
                    self._step.value = 0.0
                self._pwm_on = False
                self._last_freq = 0
            else:
                # Moving or decelerating: emit pulses at the live ramped speed.
                # Only touch the PWM when the frequency actually changes or when
                # transitioning from stopped -> moving. During a steady jog the
                # frequency is constant, so the pulse train is left completely
                # untouched and runs rock-solid (no per-cycle restart glitches).
                freq = max(1, int(round(self.current_speed)))
                if self._step is not None:
                    if freq != self._last_freq:
                        self._step.frequency = freq
                        self._last_freq = freq
                    if not self._pwm_on:
                        self._step.value = 0.5  # 50% duty -> emit step pulses
                        self._pwm_on = True
            time.sleep(RAMP_DT)

        # Worker is exiting (soft ramp-down completed). Stop pulses, de-energize
        # and clear state so the motor is fully stopped without blocking.
        self.current_speed = 0.0
        self.limit_stop = False
        self._blocked_dir = None
        if self._step is not None:
            self._step.value = 0.0  # stop emitting pulses
        self._pwm_on = False
        self._last_freq = 0
        if self._en is not None:
            self._en.off()
        with self._lock:
            self.enabled = False
            self.stopping = False

    # -- public API ---------------------------------------------------------
    def enable(self):
        """Energize the coils and HOLD position — does not move on its own.

        The motor only emits step pulses (moves) when ``run_pulses()`` is
        called (by the jog control or the playback positioner). Enabling alone
        just locks the joint in place, so clicking "Enable" never spins it.
        """
        with self._lock:
            if self.enabled:
                # Already energized; settle into a safe hold (no motion) until
                # something explicitly requests pulses (jog / playback).
                self._soft_stop.clear()
                self._pause.set()
                self.stopping = False
                return
            self.enabled = True
            self.stopping = False
            self.current_speed = 0.0
            self._apply_direction()
            if self._en is not None:
                self._en.on()  # energize (handles active-low)
            self._stop.clear()
            self._soft_stop.clear()
            self._pause.set()  # start HELD: energized but not moving
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def hold(self):
        """Keep the coils energized but stop emitting pulses (position hold).

        Used by the closed-loop positioner to freeze a joint once it reaches
        its target. Starts the worker if it isn't already running.
        """
        if self._thread is None or not self._thread.is_alive():
            self.enable()
        self._soft_stop.clear()
        self._pause.set()

    def run_pulses(self):
        """Resume emitting step pulses (closed-loop positioner is driving)."""
        self._pause.clear()

    def disable(self, soft=True):
        with self._lock:
            if not self.enabled:
                return
            if soft:
                # Non-blocking: the worker ramps to zero and cleans up itself.
                self.stopping = True
                self._soft_stop.set()
                return
            # Hard stop: cut immediately.
            self.enabled = False
            self.stopping = False
            self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
            self._thread = None
        self._soft_stop.clear()
        self._stop.clear()
        self.current_speed = 0.0
        if self._step is not None:
            self._step.value = 0.0  # stop emitting pulses
        self._pwm_on = False
        self._last_freq = 0
        if self._en is not None:
            self._en.off()  # de-energize coils

    def set_direction(self, direction):
        if direction not in ("cw", "ccw"):
            raise ValueError("direction must be 'cw' or 'ccw'")
        with self._lock:
            self.direction = direction
            self._apply_direction()

    def set_speed(self, speed):
        speed = int(speed)
        speed = max(MIN_SPEED, min(MAX_SPEED, speed))
        with self._lock:
            self.speed = speed
        return speed

    def _steps_per_rev(self):
        return max(1, self.full_steps_per_rev * self.microstepping)

    def set_rpm(self, rpm):
        # rpm is the desired OUTPUT-shaft speed; the motor must spin gear_ratio
        # times faster to achieve it.
        rpm = float(rpm)
        with self._lock:
            motor_rpm = rpm * self.gear_ratio
            speed = round(motor_rpm * self._steps_per_rev() / 60.0)
            self.speed = max(MIN_SPEED, min(MAX_SPEED, speed))
        return self.speed

    def set_geometry(self, full_steps_per_rev=None, microstepping=None):
        with self._lock:
            if full_steps_per_rev is not None:
                fs = int(full_steps_per_rev)
                if fs < 1:
                    raise ValueError("full_steps_per_rev must be >= 1")
                self.full_steps_per_rev = fs
            if microstepping is not None:
                ms = int(microstepping)
                if ms < 1:
                    raise ValueError("microstepping must be >= 1")
                self.microstepping = ms

    def set_accel(self, accel):
        accel = int(accel)
        accel = max(MIN_ACCEL, min(MAX_ACCEL, accel))
        with self._lock:
            self.accel = accel
        return accel

    def _rpm(self):
        # Reported as OUTPUT-shaft RPM (motor RPM divided by the reduction).
        return round(self.speed * 60.0 / self._steps_per_rev() / self.gear_ratio, 2)

    def _current_rpm(self):
        return round(
            self.current_speed * 60.0 / self._steps_per_rev() / self.gear_ratio, 2
        )

    def _apply_direction(self):
        if self._dir is None:
            return
        if self.direction == "cw":
            self._dir.on()
        else:
            self._dir.off()

    def _limit_pressed(self):
        """Raw state of the shared end-stop line (True = a switch is pressed)."""
        try:
            return self._limit is not None and bool(self._limit.is_active)
        except Exception:
            return False

    def _limit_pressed_cw(self):
        try:
            return self._limit_cw is not None and bool(self._limit_cw.is_active)
        except Exception:
            return False

    def _limit_pressed_ccw(self):
        try:
            return self._limit_ccw is not None and bool(self._limit_ccw.is_active)
        except Exception:
            return False

    def _limit_state(self):
        """End-stop info, or None if this motor has no limit switch."""
        if self._limit is None and self._limit_cw is None and self._limit_ccw is None:
            return None
        # Report the state the _run() thread is actually acting on, not a
        # fresh live re-read that can disagree with it and cause UI flicker.
        bd = self._blocked_dir
        return {
            "pressed": bd is not None,
            "blocked_dir": bd,
            "cw": bd == "cw",
            "ccw": bd == "ccw",
            "shared": self._limit_pressed() if self._limit is not None else False,
        }

    def status(self):
        return {
            "enabled": self.enabled,
            "stopping": self.stopping,
            "direction": self.direction,
            "speed": self.speed,
            "rpm": self._rpm(),
            "current_speed": round(self.current_speed),
            "current_rpm": self._current_rpm(),
            "accel": self.accel,
            "full_steps_per_rev": self.full_steps_per_rev,
            "microstepping": self.microstepping,
            "steps_per_rev": self._steps_per_rev(),
            "gear_ratio": self.gear_ratio,
            "limit_stop": self.limit_stop,
            "limit": self._limit_state(),
            "gpio": GPIO_AVAILABLE,
        }


motors = {
    1: StepperMotor(EN_PIN, STP_PIN, DIR_PIN, MOTOR1_GEAR_RATIO),
    2: StepperMotor(
        EN2_PIN,
        STP2_PIN,
        DIR2_PIN,
        MOTOR2_GEAR_RATIO,
        limit_pin_cw=M2_LIMIT_CW_PIN,
        limit_pin_ccw=M2_LIMIT_CCW_PIN,
        limit_cw_active_low=M2_LIMIT_CW_ACTIVE_LOW,
        limit_ccw_active_low=M2_LIMIT_CCW_ACTIVE_LOW,
    ),
    3: StepperMotor(
        EN3_PIN, STP3_PIN, DIR3_PIN, MOTOR3_GEAR_RATIO, limit_pin=M3_LIMIT_PIN,
    ),
    4: StepperMotor(
        EN4_PIN, STP4_PIN, DIR4_PIN, MOTOR4_GEAR_RATIO, limit_pin=M4_LIMIT_PIN,
    ),
}

# Standard RC servos for axis 5 & 6 using true hardware PWM (rpi-hardware-pwm).
servos = {}            # sid -> HardwarePWM instance
servo_channels = {}    # sid -> PWM channel number
servo_angles = {}
servo_enabled = {}
servo_hold = {}
servo_detach_delay = {}
servo_detach_timers = {}
servo_lock = threading.Lock()

# Axis 5 smoothing to avoid sharp mechanical shocks.
AXIS5_SMOOTH_STEP_DEG = 1.0
AXIS5_SMOOTH_STEP_SEC = 0.012


def _angle_to_pulsewidth_us(angle):
    """Convert servo angle (-135..+135 deg) to PWM pulse width in microseconds."""
    angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, float(angle)))
    # Linear interpolation: -135° -> 500μs, 0° -> 1500μs, +135° -> 2500μs
    return SERVO_CENTER_PULSE_US + (angle / 135.0) * 1000.0


def _angle_to_duty_cycle(angle):
    """Convert servo angle to PWM duty cycle percent for a 20 ms frame."""
    pw_us = _angle_to_pulsewidth_us(angle)
    return (pw_us / SERVO_PERIOD_US) * 100.0


def _set_servo_angle_hw(sid, angle):
    """Drive the servo to an angle via hardware PWM. Safe to call repeatedly."""
    pwm = servos.get(sid)
    if pwm is None:
        return
    duty = _angle_to_duty_cycle(angle)
    try:
        pwm.change_duty_cycle(duty)
    except Exception:
        pass


def _set_servo_angle_smooth(sid, start_angle, target_angle):
    """Move a servo to target in small steps.

    Axis 5 uses this to reduce abrupt motion and protect the mechanism.
    """
    start = float(start_angle)
    target = float(target_angle)
    if abs(target - start) < 0.001:
        _set_servo_angle_hw(sid, target)
        return

    step = AXIS5_SMOOTH_STEP_DEG if target > start else -AXIS5_SMOOTH_STEP_DEG
    current = start
    while True:
        nxt = current + step
        if (step > 0 and nxt >= target) or (step < 0 and nxt <= target):
            break
        _set_servo_angle_hw(sid, nxt)
        current = nxt
        time.sleep(AXIS5_SMOOTH_STEP_SEC)
    _set_servo_angle_hw(sid, target)


def _cancel_servo_detach(sid):
    t = servo_detach_timers.get(sid)
    if t is not None:
        try:
            t.cancel()
        except Exception:
            pass
    servo_detach_timers[sid] = None


def _schedule_servo_detach(sid, delay_s=None):
    """Detach after motion when hold mode is off to avoid idle jitter."""
    if delay_s is None:
        delay_s = float(servo_detach_delay.get(sid, 0.25))
    _cancel_servo_detach(sid)

    def _do_detach():
        with servo_lock:
            if not servo_enabled.get(sid, False):
                return
            if servo_hold.get(sid, True):
                return
            pwm = servos.get(sid)
            if pwm is not None:
                # 0% duty -> no pulse -> servo releases torque (idle).
                try:
                    pwm.change_duty_cycle(0)
                except Exception:
                    pass

    t = threading.Timer(delay_s, _do_detach)
    t.daemon = True
    servo_detach_timers[sid] = t
    t.start()


def _detect_servo_pwm_chip():
    """Return the sysfs pwmchip index that drives GPIO 12/13 (RP1 PWM0).

    The kernel does not guarantee a fixed pwmchip number, so match the chip by
    its hardware peripheral address instead of assuming a fixed index.
    """
    base = "/sys/class/pwm"
    try:
        for name in os.listdir(base):
            if not name.startswith("pwmchip"):
                continue
            try:
                dev = os.path.realpath(os.path.join(base, name, "device"))
            except Exception:
                continue
            if SERVO_PWM_HW_ADDR in dev:
                try:
                    return int(name.replace("pwmchip", ""))
                except ValueError:
                    continue
    except Exception:
        pass
    return SERVO_PWM_CHIP


if HW_PWM_AVAILABLE and HardwarePWM is not None:
    _pwm_chip = _detect_servo_pwm_chip()
    _servo_setup = {
        5: SERVO5_CHANNEL,
        6: SERVO6_CHANNEL,
    }
    for _sid, _chan in _servo_setup.items():
        try:
            _pwm = HardwarePWM(
                pwm_channel=_chan, hz=SERVO_PWM_HZ, chip=_pwm_chip
            )
            # Start centered at 0 degrees.
            _pwm.start(_angle_to_duty_cycle(0.0))
            servos[_sid] = _pwm
            servo_channels[_sid] = _chan
            servo_angles[_sid] = 0.0
            servo_enabled[_sid] = True
            servo_hold[_sid] = True
            servo_detach_delay[_sid] = 0.25
            servo_detach_timers[_sid] = None
        except Exception as _e:
            # PWM chip/channel unavailable (overlay not enabled or in use).
            print(
                f"[servo] init failed sid={_sid} chan={_chan} "
                f"chip={_pwm_chip}: {_e}",
                flush=True,
            )


# ---------------------------------------------------------------------------
# SERVO42C UART encoder reader
#   Wiring (Option A - shared multi-drop bus, one Pi UART):
#     Pi TXD (GPIO14) -> every SERVO42C Rx
#     every SERVO42C Tx -> Pi RXD (GPIO15)
#     Pi GND          -> every SERVO42C G
#   Each driver has a distinct address (set on its OLED): motor 1 = 0xE0
#   (slot 0), motor 2 = 0xE1 (slot 1). Only the addressed motor replies, so
#   the readers share ONE serial connection guarded by a single lock.
#   Read encoder command (manual 5.1.1): send "ADDR 30 CRC", returns 8 bytes:
#     ADDR + carry(int32, big-endian) + value(uint16, big-endian) + CRC
#   CRC is checksum-8 (sum of preceding bytes & 0xFF).
#   The encoder updates in any work mode (incl. the default CR_vFOC), so this
#   works alongside STP/DIR motion without switching to CR_UART.
# ---------------------------------------------------------------------------
SERIAL_PORT = os.environ.get("SERVO_UART", "/dev/serial0")
SERIAL_BAUD = int(os.environ.get("SERVO_BAUD", "9600"))
MOTOR1_ADDR = int(os.environ.get("SERVO_ADDR", "0xe0"), 0)
MOTOR2_ADDR = int(os.environ.get("SERVO_ADDR2", "0xe1"), 0)
MOTOR3_ADDR = int(os.environ.get("SERVO_ADDR3", "0xe2"), 0)
MOTOR4_ADDR = int(os.environ.get("SERVO_ADDR4", "0xe3"), 0)
ENCODER_COUNTS_PER_REV = 65536  # 0~0xFFFF maps to 0~360 degrees
READ_ENCODER_CMD = 0x30

# Single shared UART connection for the addressed multi-drop bus.
_serial_lock = threading.Lock()
_serial_conn = None
_serial_error = None
if not SERIAL_AVAILABLE:
    _serial_error = "pyserial not installed"
else:
    try:
        _serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)
    except Exception as exc:  # port missing / permission denied
        _serial_error = str(exc)


class EncoderReader:
    """Reads one addressed SERVO42C encoder over the shared UART bus."""

    def __init__(self, addr, gear_ratio=1.0):
        self.addr = addr & 0xFF
        self.gear_ratio = float(gear_ratio) or 1.0  # motor revs : output revs

    @property
    def available(self):
        return _serial_conn is not None

    @staticmethod
    def _checksum(data):
        return sum(data) & 0xFF

    def read(self):
        if _serial_conn is None:
            return {"available": False, "error": _serial_error}
        cmd = bytes([self.addr, READ_ENCODER_CMD])
        packet = cmd + bytes([self._checksum(cmd)])
        with _serial_lock:
            try:
                _serial_conn.reset_input_buffer()
                _serial_conn.write(packet)
                # An 8-byte reply at 9600 baud takes ~8 ms; keep the deadline
                # short so a non-responding driver fails fast instead of
                # holding the shared lock (and starving control requests).
                resp = bytearray()
                deadline = time.time() + 0.12
                while len(resp) < 8 and time.time() < deadline:
                    chunk = _serial_conn.read(8 - len(resp))
                    if chunk:
                        resp.extend(chunk)
                    elif resp:
                        # Got a partial frame then a gap: reply is done/short.
                        break
                resp = bytes(resp)
            except Exception as exc:
                return {"available": True, "error": str(exc)}

        if len(resp) != 8:
            return {"available": True,
                    "error": f"short response ({len(resp)} of 8 bytes)"}
        if resp[0] != self.addr:
            return {"available": True, "error": "unexpected address in reply"}
        if self._checksum(resp[:7]) != resp[7]:
            return {"available": True, "error": "checksum mismatch"}

        carry = int.from_bytes(resp[1:5], "big", signed=True)
        value = int.from_bytes(resp[5:7], "big", signed=False)
        counts = carry * ENCODER_COUNTS_PER_REV + value
        angle = value / ENCODER_COUNTS_PER_REV * 360.0
        total_angle = counts / ENCODER_COUNTS_PER_REV * 360.0
        revolutions = counts / ENCODER_COUNTS_PER_REV
        # The encoder is on the motor shaft; divide by the reduction to get the
        # geared OUTPUT-shaft motion.
        output_total_angle = total_angle / self.gear_ratio
        output_revolutions = revolutions / self.gear_ratio
        output_angle = output_total_angle % 360.0
        return {
            "available": True,
            "error": None,
            "carry": carry,
            "value": value,
            "counts": counts,
            "gear_ratio": self.gear_ratio,
            "angle_deg": round(angle, 2),
            "total_angle_deg": round(total_angle, 2),
            "revolutions": round(revolutions, 4),
            "output_angle_deg": round(output_angle, 2),
            "output_total_angle_deg": round(output_total_angle, 2),
            "output_revolutions": round(output_revolutions, 4),
        }


encoders = {
    1: EncoderReader(MOTOR1_ADDR, MOTOR1_GEAR_RATIO),
    2: EncoderReader(MOTOR2_ADDR, MOTOR2_GEAR_RATIO),
    3: EncoderReader(MOTOR3_ADDR, MOTOR3_GEAR_RATIO),
    4: EncoderReader(MOTOR4_ADDR, MOTOR4_GEAR_RATIO),
}


def get_motor(mid):
    return motors.get(mid)


def get_encoder(mid):
    return encoders.get(mid)


# ---- Digital twin joint-state polling ----------------------------------
# Maps URDF joints to real hardware sources. User-calibrated mapping:
#   joint1 -> Motor4, joint2 -> Motor3 (PAN), joint3 -> Motor2,
#   joint4 -> Motor1, joint5 -> Servo5 (Axis5 tilt).
# SIGN flips a joint's direction so the on-screen twin matches the real arm;
# tune these against the hardware.
TWIN_JOINT_ENCODERS = {1: 4, 2: 3, 3: 2, 4: 1}   # twin joint -> encoder/motor id
TWIN_JOINT_SERVOS = {5: 5}                       # twin joint -> servo id
TWIN_JOINT_SIGN = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0, 5: 1.0}
TWIN_POLL_DT = 0.12                              # ~8 Hz encoder cache refresh


class TwinPoller:
    """Caches the 6 joint angles (radians) for the 3D digital-twin viewer.

    The stepper encoders sit on a slow shared 9600-baud bus, so a background
    thread polls them into a cache and the HTTP endpoint just serves the cache.
    Polling only runs while the viewer has it enabled to keep the bus free for
    teach/playback.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._enabled = False
        self._thread = None
        self._stop = threading.Event()
        self._angles = {f"joint{i}": 0.0 for i in range(1, 6)}
        self._zero_rev = {mid: 0.0 for mid in TWIN_JOINT_ENCODERS.values()}
        self._zero_servo = {sid: 0.0 for sid in TWIN_JOINT_SERVOS.values()}
        self._zeroed = False
        self._errors = {}
        self._updated = 0.0

    def start(self):
        with self._lock:
            if self._enabled:
                return
            self._enabled = True
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            self._enabled = False
        self._stop.set()

    def _read_joint_angles(self):
        angles = {}
        errs = {}
        for ji, mid in TWIN_JOINT_ENCODERS.items():
            enc = get_encoder(mid)
            r = enc.read() if enc else {"error": "no encoder"}
            if r and r.get("error") is None:
                rev = r.get("output_revolutions", 0.0)
                rad = (rev - self._zero_rev[mid]) * 2.0 * math.pi * TWIN_JOINT_SIGN[ji]
                angles[f"joint{ji}"] = rad
            else:
                errs[f"joint{ji}"] = (r or {}).get("error", "read failed")
        for ji, sid in TWIN_JOINT_SERVOS.items():
            deg = float(servo_angles.get(sid, 0.0)) - self._zero_servo[sid]
            angles[f"joint{ji}"] = math.radians(deg) * TWIN_JOINT_SIGN[ji]
        return angles, errs

    def _run(self):
        while not self._stop.is_set():
            angles, errs = self._read_joint_angles()
            with self._lock:
                self._angles.update(angles)
                self._errors = errs
                self._updated = time.time()
            self._stop.wait(TWIN_POLL_DT)

    def set_zero(self):
        """Capture the current pose as the twin home (all joints -> 0)."""
        for mid in TWIN_JOINT_ENCODERS.values():
            enc = get_encoder(mid)
            r = enc.read() if enc else None
            if r and r.get("error") is None:
                self._zero_rev[mid] = r.get("output_revolutions", 0.0)
        for sid in TWIN_JOINT_SERVOS.values():
            self._zero_servo[sid] = float(servo_angles.get(sid, 0.0))
        # Refresh the cache immediately so the viewer snaps to home.
        angles, errs = self._read_joint_angles()
        with self._lock:
            self._angles.update(angles)
            self._errors = errs
            self._updated = time.time()
        self._zeroed = True
        return self.snapshot()

    def is_enabled(self):
        with self._lock:
            return self._enabled

    def is_zeroed(self):
        return self._zeroed

    def zero_rev(self, mid):
        return self._zero_rev.get(mid, 0.0)

    def zero_servo(self, sid):
        return self._zero_servo.get(sid, 0.0)

    def snapshot(self):
        with self._lock:
            return {
                "enabled": self._enabled,
                "updated": round(self._updated, 3),
                "angles": {k: round(v, 5) for k, v in self._angles.items()},
                "errors": dict(self._errors),
            }


twin_poller = TwinPoller()


# ---------------------------------------------------------------------------
# Robot arm: teach (record encoder poses) and playback (closed-loop move)
# ---------------------------------------------------------------------------
class RobotArm:
    """Coordinates the joints for teach-and-playback.

    A *waypoint* stores the absolute encoder count of every joint. Playback
    drives all joints toward each waypoint at once using software closed-loop
    position control (read encoder -> proportional speed toward target ->
    stop within a tolerance band), then dwells and advances to the next one.
    """

    def __init__(self, motors, encoders):
        self.motors = motors
        self.encoders = encoders
        # +1 if commanding "cw" makes the encoder count increase, -1 if it
        # decreases. Learned automatically on the first move of each joint and
        # reused afterwards (wiring polarity differs per motor).
        self.polarity = {mid: 1 for mid in motors}
        self._stop = threading.Event()
        self._thread = None
        self._state_lock = threading.Lock()
        self.state = {
            "playing": False,
            "freedrive": False,
            "loop": 0,
            "loops": 0,
            "waypoint": 0,
            "total": 0,
            "message": "idle",
        }

    # -- state ----------------------------------------------------------
    def _set_state(self, **kw):
        with self._state_lock:
            self.state.update(kw)

    def get_state(self):
        with self._state_lock:
            return dict(self.state)

    # -- pose capture ---------------------------------------------------
    def _read_counts(self, mid):
        d = self.encoders[mid].read()
        if d.get("error"):
            return None
        return d.get("counts")

    def capture(self):
        """Snapshot every joint's encoder position (counts + output angle)."""
        pose, angles = {}, {}
        for mid, enc in self.encoders.items():
            d = enc.read()
            if d.get("error"):
                pose[mid] = None
                angles[mid] = None
            else:
                pose[mid] = d["counts"]
                angles[mid] = d["output_total_angle_deg"]
        return {"pose": pose, "angles": angles}

    # -- free-drive (hand guiding) -------------------------------------
    def free_drive(self, on):
        for m in self.motors.values():
            if on:
                m.disable(soft=False)  # de-energize so the arm moves by hand
            else:
                m.hold()  # re-energize and hold the current position
        self._set_state(
            freedrive=bool(on),
            message="free-drive ON - move the arm by hand"
            if on
            else "joints holding position",
        )

    # -- closed-loop coordinated move ----------------------------------
    def move_to_pose(self, targets, speed_frac=1.0):
        """Drive all given joints to their target encoder counts at once."""
        active = {
            int(mid): int(t)
            for mid, t in targets.items()
            if t is not None and int(mid) in self.motors
        }
        if not active:
            return True
        speed_cap = int(
            _clamp(
                MAX_SPEED * 0.6 * _clamp(speed_frac, 0.05, 1.0),
                POS_APPROACH_MIN_SPS,
                MAX_SPEED,
            )
        )
        for mid in active:
            self.motors[mid].enable()
        last_err = {mid: None for mid in active}
        wrong = {mid: 0 for mid in active}
        confirmed = {mid: False for mid in active}
        done = set()
        deadline = time.time() + POS_TIMEOUT_S
        while time.time() < deadline and not self._stop.is_set():
            all_done = True
            for mid, target in active.items():
                m = self.motors[mid]
                cur = self._read_counts(mid)
                if cur is None:
                    m.hold()  # no feedback -> don't move blindly
                    continue
                err = target - cur
                if abs(err) <= POS_TOLERANCE_COUNTS:
                    done.add(mid)
                    m.hold()
                    continue
                all_done = False
                done.discard(mid)
                # Direction from the sign of the error and learned polarity.
                m.set_direction("cw" if (err * self.polarity[mid]) > 0 else "ccw")
                # Until the move direction is confirmed correct, cap speed so a
                # wrong-polarity guess can only nudge the joint a little.
                cap = speed_cap if confirmed[mid] else min(speed_cap, POS_SAFE_SPS)
                m.set_speed(int(_clamp(abs(err) * POS_KP, POS_APPROACH_MIN_SPS, cap)))
                m.run_pulses()
                le = last_err[mid]
                if le is not None:
                    if abs(err) < abs(le) - 4:
                        confirmed[mid] = True  # error shrinking -> right way
                        wrong[mid] = 0
                    elif abs(err) > abs(le) + 12:
                        wrong[mid] += 1
                        if wrong[mid] >= 6:  # consistently diverging -> flip
                            self.polarity[mid] *= -1
                            wrong[mid] = 0
                            confirmed[mid] = False
                last_err[mid] = err
            if all_done:
                break
            time.sleep(0.03)
        for mid in active:
            self.motors[mid].hold()
        return len(done) == len(active)

    # -- playback engine -----------------------------------------------
    def play(self, waypoints, speed_frac, loops, default_dwell):
        self._stop.clear()
        total = len(waypoints)
        loop = 0
        try:
            while not self._stop.is_set():
                loop += 1
                self._set_state(loop=loop, loops=loops)
                for idx, wp in enumerate(waypoints):
                    if self._stop.is_set():
                        break
                    self._set_state(
                        playing=True,
                        waypoint=idx + 1,
                        total=total,
                        message="moving to waypoint %d" % (idx + 1),
                    )
                    self.move_to_pose(wp.get("pose", {}), speed_frac)
                    if self._stop.is_set():
                        break
                    dwell = wp.get("dwell")
                    if dwell is None:
                        dwell = default_dwell
                    self._set_state(
                        message="dwell %.1fs at waypoint %d" % (dwell, idx + 1)
                    )
                    self._interruptible_sleep(dwell)
                if loops and loop >= loops:
                    break
        finally:
            self._set_state(
                playing=False,
                message="stopped" if self._stop.is_set() else "playback complete",
            )

    def _interruptible_sleep(self, seconds):
        end = time.time() + max(0.0, float(seconds))
        while time.time() < end and not self._stop.is_set():
            time.sleep(0.05)

    def start_playback(self, waypoints, speed_frac=1.0, loops=1, default_dwell=0.5):
        if self._thread and self._thread.is_alive():
            return False
        self._stop.clear()
        self._thread = threading.Thread(
            target=self.play,
            args=(waypoints, speed_frac, loops, default_dwell),
            daemon=True,
        )
        self._thread.start()
        return True

    def stop_playback(self):
        self._stop.set()

    # -- program persistence -------------------------------------------
    def _program_path(self, name):
        if not re.match(r"^[A-Za-z0-9 _-]{1,40}$", name or ""):
            raise ValueError("invalid program name")
        os.makedirs(PROGRAMS_DIR, exist_ok=True)
        return os.path.join(PROGRAMS_DIR, name + ".json")

    def list_programs(self):
        if not os.path.isdir(PROGRAMS_DIR):
            return []
        return sorted(
            f[:-5] for f in os.listdir(PROGRAMS_DIR) if f.endswith(".json")
        )

    def load_program(self, name):
        with open(self._program_path(name), "r") as f:
            return json.load(f)

    def save_program(self, name, data):
        with open(self._program_path(name), "w") as f:
            json.dump(data, f, indent=2)

    def delete_program(self, name):
        path = self._program_path(name)
        if os.path.exists(path):
            os.remove(path)


arm = RobotArm(motors, encoders)


# ---------------------------------------------------------------------------
# Digital twin: drive the REAL arm from the 3D model (Pose & Send)
# ---------------------------------------------------------------------------
# Which twin joints command hardware. joint1-4 are steppers (closed-loop
# encoder move); joint5 is the Axis5 tilt servo.
TWIN_COMMAND_STEPPERS = {1: 4, 2: 3, 3: 2, 4: 1}   # twin joint -> motor id
TWIN_COMMAND_SERVOS = {5: 5}                       # twin joint -> servo id (tilt)
# Per-joint travel limits in RADIANS (mirror the URDF defaults). The UI can
# override these via POST /twin/limits and the move command clamps to them.
TWIN_JOINT_LIMITS = {
    1: [-1.5708, 1.5708],
    2: [-1.5708, 1.5708],
    3: [-1.5708, 1.5708],
    4: [-1.0472, 1.0472],
    5: [-1.0472, 1.0472],
}
_twin_move_lock = threading.Lock()
_twin_move_state = {"moving": False, "ok": None, "message": "idle"}


def _twin_move_status():
    with _twin_move_lock:
        return dict(_twin_move_state)


def _twin_clamp_rad(ji, rad):
    lo, hi = TWIN_JOINT_LIMITS.get(ji, [-math.pi, math.pi])
    return _clamp(float(rad), lo, hi)


def _twin_angle_to_counts(ji, mid, rad):
    """Convert a twin joint angle (rad, relative to home) to encoder counts."""
    enc = get_encoder(mid)
    gear = float(getattr(enc, "gear_ratio", 1.0) or 1.0) if enc else 1.0
    sign = TWIN_JOINT_SIGN.get(ji, 1.0)
    target_rev = twin_poller.zero_rev(mid) + (rad * sign) / (2.0 * math.pi)
    return int(round(target_rev * ENCODER_COUNTS_PER_REV * gear))


def _twin_execute(angles, speed_frac):
    # Pause the encoder poller so the move owns the shared 9600-baud bus.
    resume_poll = twin_poller.is_enabled()
    if resume_poll:
        twin_poller.stop()
    try:
        stepper_targets = {}
        for ji, mid in TWIN_COMMAND_STEPPERS.items():
            key = f"joint{ji}"
            if key in angles:
                rad = _twin_clamp_rad(ji, angles[key])
                stepper_targets[mid] = _twin_angle_to_counts(ji, mid, rad)
        servo_moves = []
        for ji, sid in TWIN_COMMAND_SERVOS.items():
            key = f"joint{ji}"
            if key in angles:
                rad = _twin_clamp_rad(ji, angles[key])
                sign = TWIN_JOINT_SIGN.get(ji, 1.0)
                deg = twin_poller.zero_servo(sid) + math.degrees(rad) * sign
                deg = _clamp(deg, float(SERVO_MIN_ANGLE), float(SERVO_MAX_ANGLE))
                servo_moves.append((sid, deg))

        for sid, deg in servo_moves:
            prev = float(servo_angles.get(sid, 0.0))
            _set_servo_angle_smooth(sid, prev, deg)
            servo_angles[sid] = deg

        ok = True
        if stepper_targets:
            ok = arm.move_to_pose(stepper_targets, speed_frac=speed_frac)
        with _twin_move_lock:
            _twin_move_state.update(
                moving=False, ok=bool(ok),
                message="move complete" if ok
                else "move incomplete (check encoders/limits)")
    except Exception as exc:  # noqa: BLE001 - report any hardware fault
        with _twin_move_lock:
            _twin_move_state.update(moving=False, ok=False,
                                    message=f"error: {exc}")
    finally:
        if resume_poll:
            twin_poller.start()


def twin_move(angles, speed_frac):
    """Start a Pose-&-Send move toward the given twin joint angles (radians)."""
    if not twin_poller.is_zeroed():
        return False, "set home first (press Set home with the arm at zero)"
    st = arm.get_state()
    if st.get("playing") or st.get("freedrive"):
        return False, "arm busy (playback or free-drive active)"
    with _twin_move_lock:
        if _twin_move_state["moving"]:
            return False, "already moving"
        _twin_move_state.update(moving=True, ok=None, message="moving\u2026")
    arm._stop.clear()
    threading.Thread(target=_twin_execute, args=(dict(angles), speed_frac),
                     daemon=True).start()
    return True, "moving"


# ---------------------------------------------------------------------------
# Camera + on-device emotion detection
# ---------------------------------------------------------------------------
CAMERA_DEVICE = os.environ.get("MIRO_CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
# Frame-rate ceiling while any stepper is moving. Lower = less CPU contention
# with the software-timed step PWM, which keeps the motors from vibrating.
FACE_TRACK_CAM_FPS_BUSY = float(os.environ.get("FACE_TRACK_CAM_FPS_BUSY", "6"))
_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FER_MODEL_PATH = os.path.join(_MODELS_DIR, "emotion-ferplus-8.onnx")
YUNET_MODEL_PATH = os.path.join(_MODELS_DIR, "face_detection_yunet_2023mar.onnx")

# FER+ raw output order (8 classes).
_FERPLUS_LABELS = [
    "neutral", "happiness", "surprise", "sadness",
    "anger", "disgust", "fear", "contempt",
]
# The UI shows six emotions; fold the two extras onto the nearest shown class
# (disgust -> angry, contempt -> neutral).
_FERPLUS_TO_UI = {
    "neutral": "neutral", "happiness": "happy", "surprise": "surprise",
    "sadness": "sad", "anger": "angry", "disgust": "angry",
    "fear": "fear", "contempt": "neutral",
}
UI_EMOTIONS = ["happy", "neutral", "surprise", "sad", "angry", "fear"]

# Addressable LED rings (2 x WS2812B, 8 LEDs each) sharing one data line.
# On the Raspberry Pi 5 the classic DMA/PWM drivers (rpi_ws281x, pigpio) do not
# work because the GPIO block moved to the RP1 chip. The reliable method is to
# clock the WS2812 protocol out of the SPI peripheral (MOSI = GPIO10, pin 19).
RING_PIN = 10                 # SPI0 MOSI (physical pin 19) when using the SPI backend
RING_LED_COUNT = 8
RING_BRIGHTNESS = 80          # 0-255 global brightness ceiling
RING_SPI_BUS = 0
RING_SPI_DEV = 0
RING_SPI_HZ = 2400000         # 3 SPI bits per WS2812 bit -> ~417 ns/bit


class EmotionCamera:
    """USB camera capture + face detection + FER+ emotion inference.

    A single background thread grabs frames from the USB camera, finds the
    largest face (YuNet, with a Haar-cascade fallback), classifies its emotion
    with the FER+ ONNX model via ``cv2.dnn``, draws an annotation and keeps the
    latest JPEG (for the MJPEG stream) plus the latest structured result (for
    ``/emotion/latest``). The camera opens lazily on first access and releases
    itself after a period with no viewers so it is not held unnecessarily.
    """

    IDLE_TIMEOUT_S = 20.0

    def __init__(self, device=CAMERA_DEVICE, width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        self.device = device
        self.width = width
        self.height = height
        self.available = CV2_AVAILABLE and os.path.exists(FER_MODEL_PATH)
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._cap = None
        self._face_net = None
        self._emo_net = None
        self._haar = None
        self._jpeg = None
        self._result = self._empty_result()
        self._last_active = 0.0
        self._fps = 0.0
        self._emo_enabled = True    # emotion detection on by default
        self._last_scores = None    # reused when emotion inference is skipped

    @staticmethod
    def _empty_result():
        return {
            "live": False, "face": None, "box": None,
            "scores": {k: 0.0 for k in UI_EMOTIONS},
            "dominant": None, "infer_ms": 0.0, "fps": 0.0,
        }

    # -- model / capture setup ------------------------------------------
    def _load_models(self):
        if self._emo_net is None and os.path.exists(FER_MODEL_PATH):
            self._emo_net = cv2.dnn.readNetFromONNX(FER_MODEL_PATH)
        if (self._face_net is None and hasattr(cv2, "FaceDetectorYN")
                and os.path.exists(YUNET_MODEL_PATH)):
            try:
                self._face_net = cv2.FaceDetectorYN.create(
                    YUNET_MODEL_PATH, "", (self.width, self.height),
                    score_threshold=0.7, nms_threshold=0.3, top_k=50,
                )
            except Exception:
                self._face_net = None
        if self._face_net is None and self._haar is None:
            cascade = os.path.join(
                cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
            self._haar = cv2.CascadeClassifier(cascade)

    def _open_capture(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def ensure_started(self):
        self._last_active = time.time()
        if not self.available:
            return False
        with self._lock:
            if self._running:
                return True
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return True

    # -- inference ------------------------------------------------------
    def _detect_face(self, frame, gray):
        if self._face_net is not None:
            self._face_net.setInputSize((frame.shape[1], frame.shape[0]))
            _, faces = self._face_net.detect(frame)
            if faces is None or len(faces) == 0:
                return None
            best = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = int(best[0]), int(best[1]), int(best[2]), int(best[3])
        else:
            rects = self._haar.detectMultiScale(gray, 1.2, 5, minSize=(60, 60))
            if len(rects) == 0:
                return None
            x, y, w, h = max(rects, key=lambda r: r[2] * r[3])
            x, y, w, h = int(x), int(y), int(w), int(h)
        x = max(0, x)
        y = max(0, y)
        w = min(w, frame.shape[1] - x)
        h = min(h, frame.shape[0] - y)
        if w <= 0 or h <= 0:
            return None
        return (x, y, w, h)

    def _classify(self, gray, box):
        x, y, w, h = box
        roi = gray[y:y + h, x:x + w]
        if roi.size == 0:
            return None
        face = cv2.resize(roi, (64, 64)).astype(np.float32)
        blob = face.reshape(1, 1, 64, 64)
        self._emo_net.setInput(blob)
        out = self._emo_net.forward().flatten()
        ex = np.exp(out - np.max(out))
        probs = ex / ex.sum()
        ui = {k: 0.0 for k in UI_EMOTIONS}
        for label, p in zip(_FERPLUS_LABELS, probs):
            ui[_FERPLUS_TO_UI[label]] += float(p)
        total = sum(ui.values()) or 1.0
        return {k: v / total for k, v in ui.items()}

    def set_emotion(self, on):
        """Enable/disable face + emotion classification."""
        self._emo_enabled = bool(on)
        return self._emo_enabled

    # -- worker ---------------------------------------------------------
    def _run(self):
        try:
            self._load_models()
        except Exception:
            pass
        self._cap = self._open_capture()
        t_fps = time.time()
        n = 0
        fail = 0
        while self._running:
            if time.time() - self._last_active > self.IDLE_TIMEOUT_S:
                break
            ok, frame = (self._cap.read() if self._cap else (False, None))
            if not ok or frame is None:
                fail += 1
                if fail > 30:
                    try:
                        self._cap.release()
                    except Exception:
                        pass
                    time.sleep(0.5)
                    self._cap = self._open_capture()
                    fail = 0
                time.sleep(0.05)
                continue
            fail = 0
            t0 = time.time()
            emo_on = self._emo_enabled
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if emo_on else None
            box = None
            scores = None
            if emo_on:
                try:
                    box = self._detect_face(frame, gray)
                    if box is not None and self._emo_net is not None:
                        # While a stepper is moving, skip the heavier FER+
                        # emotion inference and reuse the last scores so the
                        # software step-PWM keeps clean timing (less vibration).
                        # The face box is still detected every frame for
                        # tracking.
                        try:
                            busy = any(getattr(m, "current_speed", 0) > 0
                                       for m in motors.values())
                        except Exception:
                            busy = False
                        if busy and self._last_scores is not None:
                            scores = self._last_scores
                        else:
                            scores = self._classify(gray, box)
                            self._last_scores = scores
                except Exception:
                    box = None
                    scores = None
            infer_ms = (time.time() - t0) * 1000.0

            if box is not None:
                x, y, w, h = box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (105, 211, 0), 2)
                if scores:
                    dom = max(scores, key=scores.get)
                    label = "%s %d%%" % (dom, round(scores[dom] * 100))
                    cv2.rectangle(frame, (x, max(0, y - 22)),
                                  (x + max(120, w), y), (23, 31, 12), -1)
                    cv2.putText(frame, label, (x + 6, max(14, y - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (194, 230, 105), 2)

            n += 1
            if time.time() - t_fps >= 1.0:
                self._fps = n / (time.time() - t_fps)
                n = 0
                t_fps = time.time()

            ok2, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            with self._lock:
                if ok2:
                    self._jpeg = buf.tobytes()
                if scores is not None:
                    dom = max(scores, key=scores.get)
                    self._result = {
                        "live": True, "face": 1, "emo_enabled": True,
                        "box": [box[0], box[1], box[2], box[3]],
                        "scores": scores, "dominant": dom,
                        "infer_ms": round(infer_ms, 1),
                        "fps": round(self._fps, 1),
                    }
                else:
                    self._result = {
                        "live": True, "face": None,
                        "emo_enabled": self._emo_enabled,
                        "box": None,
                        "scores": {k: 0.0 for k in UI_EMOTIONS},
                        "dominant": None, "infer_ms": round(infer_ms, 1),
                        "fps": round(self._fps, 1),
                    }
            target_fps = 15.0
            # While a stepper is actively moving (jog or face tracking) the
            # software-PWM step pulses are sensitive to CPU contention, so cap
            # the camera frame rate harder to leave the cores free and keep the
            # motors smooth.
            try:
                if any(getattr(m, "current_speed", 0) > 0 for m in motors.values()):
                    target_fps = min(target_fps, FACE_TRACK_CAM_FPS_BUSY)
            except Exception:
                pass
            frame_interval = 1.0 / target_fps
            dt = time.time() - t0
            if dt < frame_interval:
                time.sleep(frame_interval - dt)

        with self._lock:
            self._running = False
            self._jpeg = None
            self._result = self._empty_result()
        try:
            if self._cap:
                self._cap.release()
        except Exception:
            pass
        self._cap = None

    # -- accessors ------------------------------------------------------
    def latest_result(self):
        self.ensure_started()
        with self._lock:
            return dict(self._result)

    def peek_result(self):
        """Return the latest emotion result without starting camera capture."""
        with self._lock:
            return dict(self._result)

    def snapshot(self):
        if not self.ensure_started():
            return None
        for _ in range(40):
            with self._lock:
                if self._jpeg is not None:
                    return self._jpeg
            time.sleep(0.05)
        return None

    def frames(self):
        if not self.ensure_started():
            return
        boundary = b"--frame"
        while True:
            self._last_active = time.time()
            with self._lock:
                jpg = self._jpeg
                running = self._running
            if not running:
                break
            if jpg is None:
                time.sleep(0.05)
                continue
            yield (boundary + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            time.sleep(1.0 / 20.0)


camera = EmotionCamera()


# ---------------------------------------------------------------------------
# Head tracking (camera on robot head)
#   - Face-box X/Y error is mixed across all arm steppers (Motors 1-4)
#   - Axis 5 servo provides fine vertical tilt
# ---------------------------------------------------------------------------
FACE_TRACK_SERVO_ID = int(os.environ.get("FACE_TRACK_SERVO_ID", "5"))
FACE_TRACK_MOTOR_IDS = [
    int(x.strip()) for x in os.environ.get("FACE_TRACK_MOTOR_IDS", "1,2,3,4").split(",")
    if x.strip()
]
# Per-motor mix from pixel error -> virtual command:
#   cmd = ex*mix_x + ey*mix_y
# Positive cmd follows FACE_TRACK_MOTOR_SIGN to choose CW/CCW.
FACE_TRACK_MIX_X = {
    1: float(os.environ.get("FACE_TRACK_M1_MIX_X", "0.80")),
    2: float(os.environ.get("FACE_TRACK_M2_MIX_X", "0.90")),
    3: float(os.environ.get("FACE_TRACK_M3_MIX_X", "0.88")),
    4: float(os.environ.get("FACE_TRACK_M4_MIX_X", "0.82")),
}
FACE_TRACK_MIX_Y = {
    1: float(os.environ.get("FACE_TRACK_M1_MIX_Y", "0.90")),
    2: float(os.environ.get("FACE_TRACK_M2_MIX_Y", "0.40")),
    3: float(os.environ.get("FACE_TRACK_M3_MIX_Y", "0.22")),
    4: float(os.environ.get("FACE_TRACK_M4_MIX_Y", "-0.30")),
}
# Circular/orbital motion coupling. Positive values make a motor react to
# rotational head movement around the image center, not only pure X/Y drift.
# Joint 4 is Motor 1 in this rig, so M1 gets the strongest circular coupling.
FACE_TRACK_MIX_ROT = {
    1: float(os.environ.get("FACE_TRACK_M1_MIX_ROT", "1.00")),
    2: float(os.environ.get("FACE_TRACK_M2_MIX_ROT", "0.25")),
    3: float(os.environ.get("FACE_TRACK_M3_MIX_ROT", "0.10")),
    4: float(os.environ.get("FACE_TRACK_M4_MIX_ROT", "-0.15")),
}
FACE_TRACK_MOTOR_SIGN = {
    1: int(os.environ.get("FACE_TRACK_M1_SIGN", "1")),
    2: int(os.environ.get("FACE_TRACK_M2_SIGN", "1")),
    3: int(os.environ.get("FACE_TRACK_M3_SIGN", "1")),
    4: int(os.environ.get("FACE_TRACK_M4_SIGN", "1")),
}
FACE_TRACK_LOOP_DT = float(os.environ.get("FACE_TRACK_LOOP_DT", "0.06"))
FACE_TRACK_X_DEADZONE_PX = int(os.environ.get("FACE_TRACK_X_DEADZONE_PX", "45"))
FACE_TRACK_X_START_PX = int(os.environ.get("FACE_TRACK_X_START_PX", "60"))
FACE_TRACK_Y_DEADZONE_PX = int(os.environ.get("FACE_TRACK_Y_DEADZONE_PX", "35"))
# Start tilt only once the vertical error exceeds this (must be > deadzone) so
# the head does not chatter up/down around the centred position (hysteresis).
FACE_TRACK_Y_START_PX = int(os.environ.get("FACE_TRACK_Y_START_PX", "55"))
# Bias the vertical target a bit lower in the frame so upward body motion
# (standing up) creates a stronger corrective response.
FACE_TRACK_Y_TARGET_OFFSET_PX = int(os.environ.get("FACE_TRACK_Y_TARGET_OFFSET_PX", "24"))
# Directional hysteresis: upward face motion should engage sooner than
# downward motion so standing up is followed promptly.
FACE_TRACK_Y_UP_DEADZONE_PX = int(os.environ.get("FACE_TRACK_Y_UP_DEADZONE_PX", "20"))
FACE_TRACK_Y_UP_START_PX = int(os.environ.get("FACE_TRACK_Y_UP_START_PX", "30"))
# Low-pass the vertical error to reject per-frame jitter that otherwise drives
# tilt oscillation, same idea as the pan X filter.
FACE_TRACK_Y_FILTER_ALPHA = float(os.environ.get("FACE_TRACK_Y_FILTER_ALPHA", "0.4"))
FACE_TRACK_MIN_SPS = int(os.environ.get("FACE_TRACK_MIN_SPS", "260"))
FACE_TRACK_MAX_SPS = int(os.environ.get("FACE_TRACK_MAX_SPS", "1400"))
FACE_TRACK_KP_SPS_PER_PX = float(os.environ.get("FACE_TRACK_KP_SPS_PER_PX", "3.2"))
# Absolute ceiling the user speed multiplier can never push pan beyond, so a
# high "speed" setting cannot command an unsafe step rate for the motor.
FACE_TRACK_MAX_SPS_HARD = int(os.environ.get("FACE_TRACK_MAX_SPS_HARD", "2000"))
FACE_TRACK_ASSIST_MAX_RATIO = float(os.environ.get("FACE_TRACK_ASSIST_MAX_RATIO", "0.70"))
FACE_TRACK_ASSIST_MIN_RATIO = float(os.environ.get("FACE_TRACK_ASSIST_MIN_RATIO", "0.60"))
# Only push a new pan speed to the motor when it changes by at least this many
# steps/s. Constant micro-adjustments rewrite the PWM frequency every loop and
# cause the pulse train to stutter (visible as bouncing), so we quantize.
FACE_TRACK_SPS_STEP = int(os.environ.get("FACE_TRACK_SPS_STEP", "60"))
FACE_TRACK_X_FILTER_ALPHA = float(os.environ.get("FACE_TRACK_X_FILTER_ALPHA", "0.35"))
FACE_TRACK_ORBIT_FILTER_ALPHA = float(os.environ.get("FACE_TRACK_ORBIT_FILTER_ALPHA", "0.45"))
FACE_TRACK_ACT_SCALE = {
    1: float(os.environ.get("FACE_TRACK_M1_ACT_SCALE", "0.50")),
    2: float(os.environ.get("FACE_TRACK_M2_ACT_SCALE", "0.80")),
    3: float(os.environ.get("FACE_TRACK_M3_ACT_SCALE", "1.00")),
    4: float(os.environ.get("FACE_TRACK_M4_ACT_SCALE", "0.85")),
}
# Lower bound for per-joint activation scaling derived from its X/Y mix.
# Prevents weakly-weighted assist joints from never crossing the static
# start-zone while still filtering tiny camera jitter.
FACE_TRACK_MIX_ACT_MIN = float(os.environ.get("FACE_TRACK_MIX_ACT_MIN", "0.35"))
FACE_TRACK_TILT_SIGN = int(os.environ.get("FACE_TRACK_TILT_SIGN", "-1"))
FACE_TRACK_TILT_KP_DEG_PER_PX = float(os.environ.get("FACE_TRACK_TILT_KP_DEG_PER_PX", "0.03"))
# Hard cap on how many degrees the tilt servo may move per control loop so a
# large vertical error cannot fling the head; keeps tilt smooth and gentle.
# The user tilt-speed multiplier scales THIS slew-rate cap (not the gain) so a
# faster setting reaches a far face quicker without overshooting near centre.
FACE_TRACK_TILT_MAX_STEP_DEG = float(os.environ.get("FACE_TRACK_TILT_MAX_STEP_DEG", "2.6"))
# Directional tilt gain/cap scaling for upward tracking (negative ey).
FACE_TRACK_TILT_UP_GAIN = float(os.environ.get("FACE_TRACK_TILT_UP_GAIN", "1.72"))
FACE_TRACK_TILT_UP_CAP_SCALE = float(os.environ.get("FACE_TRACK_TILT_UP_CAP_SCALE", "1.85"))
FACE_TRACK_TILT_MIN = float(os.environ.get("FACE_TRACK_TILT_MIN", "-85"))
FACE_TRACK_TILT_MAX = float(os.environ.get("FACE_TRACK_TILT_MAX", "85"))
FACE_TRACK_LOST_HOLD_S = float(os.environ.get("FACE_TRACK_LOST_HOLD_S", "0.8"))
FACE_TRACK_PREP_SPEED = int(os.environ.get("FACE_TRACK_PREP_SPEED", "400"))
FACE_TRACK_PREP_ACCEL = int(os.environ.get("FACE_TRACK_PREP_ACCEL", "2500"))
# User-selectable speed multipliers (1.0 = the tuned defaults above). The pan
# multiplier scales both the proportional gain and the max step rate; the tilt
# multiplier scales the tilt gain and per-loop cap. They are clamped to the
# [MIN, MAX] range so the UI slider cannot push the rig past safe limits.
FACE_TRACK_SPEED_MIN = float(os.environ.get("FACE_TRACK_SPEED_MIN", "0.25"))
FACE_TRACK_SPEED_MAX = float(os.environ.get("FACE_TRACK_SPEED_MAX", "2.5"))
FACE_TRACK_PAN_SPEED = float(os.environ.get("FACE_TRACK_PAN_SPEED", "1.0"))
FACE_TRACK_TILT_SPEED = float(os.environ.get("FACE_TRACK_TILT_SPEED", "1.0"))


class FaceTracker:
    """Closed-loop face tracker driven by the camera face box."""

    def __init__(self, camera):
        self.camera = camera
        self.enabled = False
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._state = {
            "live": False,
            "has_face": False,
            "face_center": None,
            "error_px": [0, 0],
            "motor_ids": list(FACE_TRACK_MOTOR_IDS),
            "servo_id": FACE_TRACK_SERVO_ID,
            "motor_ready": False,
            "servo_ready": False,
            "motor_sps": 0,
            "motor_sps_map": {},
            "servo_angle": 0.0,
            "updated": 0.0,
            "error": None,
        }
        self._motor_active = {mid: False for mid in FACE_TRACK_MOTOR_IDS}
        self._motor_err_f = {mid: 0.0 for mid in FACE_TRACK_MOTOR_IDS}
        self._motor_dir = {mid: None for mid in FACE_TRACK_MOTOR_IDS}
        self._motor_last_sps = {mid: 0 for mid in FACE_TRACK_MOTOR_IDS}
        self._saved_motor_state = {}
        self._prev_ex = None
        self._prev_ey = None
        self._orbit_err_f = 0.0
        # Filtered vertical error + tilt hysteresis state (anti-oscillation).
        self._tilt_err_f = 0.0
        self._tilt_active = False
        # Runtime tilt travel limits. Default to the safe tracker range but the
        # UI "Hard Limit" field overrides these so tilt can never drive the
        # servo into its mechanical end-stops.
        self._tilt_min = float(FACE_TRACK_TILT_MIN)
        self._tilt_max = float(FACE_TRACK_TILT_MAX)
        self._state["tilt_min"] = self._tilt_min
        self._state["tilt_max"] = self._tilt_max
        # User-selectable speed multipliers (clamped). 1.0 == tuned defaults.
        self._pan_speed = self._clamp_speed(FACE_TRACK_PAN_SPEED)
        self._tilt_speed = self._clamp_speed(FACE_TRACK_TILT_SPEED)
        self._state["pan_speed"] = self._pan_speed
        self._state["tilt_speed"] = self._tilt_speed

    @staticmethod
    def _clamp_speed(value):
        try:
            v = float(value)
        except (TypeError, ValueError):
            v = 1.0
        return round(max(FACE_TRACK_SPEED_MIN, min(FACE_TRACK_SPEED_MAX, v)), 3)

    def set_speed(self, pan=None, tilt=None):
        """Set the pan/tilt speed multipliers (None leaves a value unchanged)."""
        with self._lock:
            if pan is not None:
                self._pan_speed = self._clamp_speed(pan)
                self._state["pan_speed"] = self._pan_speed
            if tilt is not None:
                self._tilt_speed = self._clamp_speed(tilt)
                self._state["tilt_speed"] = self._tilt_speed
            return {
                "pan_speed": self._pan_speed,
                "tilt_speed": self._tilt_speed,
            }

    def set_tilt_limits(self, min_deg=None, max_deg=None):
        """Set the tilt travel limits (degrees), clamped to the servo's
        physical range and kept ordered. None leaves a value unchanged."""
        with self._lock:
            lo = self._tilt_min if min_deg is None else min_deg
            hi = self._tilt_max if max_deg is None else max_deg
            try:
                lo = float(lo)
                hi = float(hi)
            except (TypeError, ValueError):
                return {"tilt_min": self._tilt_min, "tilt_max": self._tilt_max}
            lo = max(float(SERVO_MIN_ANGLE), min(float(SERVO_MAX_ANGLE), lo))
            hi = max(float(SERVO_MIN_ANGLE), min(float(SERVO_MAX_ANGLE), hi))
            if lo > hi:
                lo, hi = hi, lo
            self._tilt_min = round(lo, 2)
            self._tilt_max = round(hi, 2)
            self._state["tilt_min"] = self._tilt_min
            self._state["tilt_max"] = self._tilt_max
            return {"tilt_min": self._tilt_min, "tilt_max": self._tilt_max}

    def _tracked_motors(self):
        return {mid: motors[mid] for mid in FACE_TRACK_MOTOR_IDS if mid in motors}

    def _save_motor_state_if_needed(self, mid, motor):
        if motor is None or mid in self._saved_motor_state:
            return
        self._saved_motor_state[mid] = {
            "speed": int(getattr(motor, "speed", DEFAULT_SPEED)),
            "direction": getattr(motor, "direction", "cw"),
            "accel": int(getattr(motor, "accel", DEFAULT_ACCEL)),
            "enabled": bool(getattr(motor, "enabled", False)),
        }

    def _restore_motor_state(self, mid, motor):
        if motor is None or mid not in self._saved_motor_state:
            return
        saved = self._saved_motor_state.get(mid, {})
        try:
            motor.set_speed(saved.get("speed", DEFAULT_SPEED))
        except Exception:
            pass
        try:
            motor.set_direction(saved.get("direction", "cw"))
        except Exception:
            pass
        try:
            motor.set_accel(saved.get("accel", DEFAULT_ACCEL))
        except Exception:
            pass
        try:
            if not saved.get("enabled", False):
                motor.disable(soft=True)
        except Exception:
            pass
        self._saved_motor_state.pop(mid, None)

    def _prepare_outputs(self):
        motor_ready = False
        servo_ready = False

        tracked = self._tracked_motors()
        for mid, m in tracked.items():
            try:
                self._save_motor_state_if_needed(mid, m)
                # Preload tracker motion params but DO NOT enable/hold here;
                # ownership stays with jog/manual control until a face command
                # actually asks this motor to move.
                m.set_accel(FACE_TRACK_PREP_ACCEL)
                m.set_speed(FACE_TRACK_PREP_SPEED)
                motor_ready = True
            except Exception:
                continue

        if FACE_TRACK_SERVO_ID in servos:
            with servo_lock:
                try:
                    servo_enabled[FACE_TRACK_SERVO_ID] = True
                    _cancel_servo_detach(FACE_TRACK_SERVO_ID)
                    angle = float(servo_angles.get(FACE_TRACK_SERVO_ID, 0.0))
                    _set_servo_angle_hw(FACE_TRACK_SERVO_ID, angle)
                    servo_ready = True
                except Exception:
                    servo_ready = False

        self._set_state(motor_ready=motor_ready, servo_ready=servo_ready)

    def start(self):
        with self._lock:
            self.enabled = True
        self._prepare_outputs()
        with self._lock:
            if self._running:
                return True
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return True

    def stop(self):
        with self._lock:
            self.enabled = False
        self._stop_outputs()
        return True

    def status(self):
        with self._lock:
            st = dict(self._state)
            st["enabled"] = bool(self.enabled)
        return st

    def _set_state(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
            self._state["updated"] = round(time.time(), 3)

    def _stop_outputs(self, restore_motor=True, mark_not_ready=True):
        tracked = self._tracked_motors()
        for mid, m in tracked.items():
            # Only force HOLD for motors currently driven by the tracker.
            # Calling hold() on inactive motors can steal them from jog control.
            if self._motor_active.get(mid, False):
                try:
                    m.hold()
                except Exception:
                    pass
            if restore_motor:
                self._restore_motor_state(mid, m)
            self._motor_active[mid] = False
            self._motor_err_f[mid] = 0.0
            self._motor_dir[mid] = None
            self._motor_last_sps[mid] = 0
        self._prev_ex = None
        self._prev_ey = None
        self._orbit_err_f = 0.0
        self._tilt_err_f = 0.0
        self._tilt_active = False
        if mark_not_ready:
            self._set_state(motor_ready=False, servo_ready=bool(
                FACE_TRACK_SERVO_ID in servos and servo_enabled.get(FACE_TRACK_SERVO_ID, False)
            ))

    def _run(self):
        last_face_ts = 0.0
        was_enabled = False
        while True:
            with self._lock:
                running = self._running
                enabled = self.enabled
                pan_speed = self._pan_speed
                tilt_speed = self._tilt_speed
                tilt_min = self._tilt_min
                tilt_max = self._tilt_max
            if not running:
                break

            if not enabled:
                # Release the outputs ONCE on the enabled->disabled transition,
                # then idle. Calling _stop_outputs() every loop would keep
                # asserting hold() on the tracked motor, which permanently
                # blocks manual jog and any other use of that motor.
                if was_enabled:
                    self._set_state(live=False, has_face=False, motor_sps=0)
                    self._stop_outputs()
                    was_enabled = False
                time.sleep(FACE_TRACK_LOOP_DT)
                continue
            was_enabled = True

            # Ensure face detector stays on while tracking.
            self.camera.set_emotion(True)
            d = self.camera.latest_result()
            box = d.get("box")
            has_face = d.get("face") is not None and isinstance(box, list) and len(box) == 4
            if has_face:
                last_face_ts = time.time()
                x, y, w, h = [int(v) for v in box]
                cx = x + (w // 2)
                cy = y + (h // 2)
                ex = int(cx - (CAMERA_WIDTH // 2))
                ey = int(cy - (CAMERA_HEIGHT // 2))

                # Multi-joint head tracking: all configured motors participate
                # using a weighted X/Y error mix per motor.
                motor_sps_map = {}
                tracked = self._tracked_motors()
                alpha = max(0.0, min(1.0, FACE_TRACK_X_FILTER_ALPHA))
                orbit_alpha = max(0.0, min(1.0, FACE_TRACK_ORBIT_FILTER_ALPHA))
                stop_zone = max(0, int(FACE_TRACK_X_DEADZONE_PX))
                start_zone = max(stop_zone + 1, int(FACE_TRACK_X_START_PX))

                # Orbital component: detects circular head motion around frame
                # center via the signed tangent speed term.
                if self._prev_ex is None or self._prev_ey is None:
                    d_ex, d_ey = 0.0, 0.0
                else:
                    d_ex = float(ex - self._prev_ex)
                    d_ey = float(ey - self._prev_ey)
                radius = max(1.0, math.hypot(float(ex), float(ey)))
                orbit_raw = ((float(ex) * d_ey) - (float(ey) * d_ex)) / radius
                self._orbit_err_f = ((1.0 - orbit_alpha) * self._orbit_err_f) + (orbit_alpha * orbit_raw)
                self._prev_ex = ex
                self._prev_ey = ey

                for mid, m in tracked.items():
                    try:
                        mix_x = float(FACE_TRACK_MIX_X.get(mid, 0.0))
                        mix_y = float(FACE_TRACK_MIX_Y.get(mid, 0.0))
                        mix_r = float(FACE_TRACK_MIX_ROT.get(mid, 0.0))
                        mix_mag = max(
                            FACE_TRACK_MIX_ACT_MIN,
                            min(1.0, abs(mix_x) + abs(mix_y) + (0.5 * abs(mix_r))),
                        )
                        act_scale = max(0.45, min(1.10, float(FACE_TRACK_ACT_SCALE.get(mid, 1.0))))
                        cmd_raw = (float(ex) * mix_x) + (float(ey) * mix_y) + (self._orbit_err_f * mix_r)
                        self._motor_err_f[mid] = (
                            (1.0 - alpha) * self._motor_err_f.get(mid, 0.0)
                            + alpha * cmd_raw
                        )
                        cmd = int(round(self._motor_err_f[mid]))

                        # Scale per-joint hysteresis by mix magnitude so all
                        # configured joints can engage, not only the dominant pan motor.
                        stop_zone_m = max(8, int(round(stop_zone * mix_mag * act_scale)))
                        start_zone_m = max(stop_zone_m + 1, int(round(start_zone * mix_mag)))
                        start_zone_m = max(stop_zone_m + 1, int(round(start_zone * mix_mag * act_scale)))

                        if self._motor_active.get(mid, False):
                            should_move = abs(cmd) > stop_zone_m
                        else:
                            should_move = abs(cmd) >= start_zone_m

                        if not should_move:
                            if self._motor_active.get(mid, False):
                                try:
                                    m.hold()
                                except Exception:
                                    pass
                            self._motor_active[mid] = False
                            self._motor_dir[mid] = None
                            self._motor_last_sps[mid] = 0
                            motor_sps_map[str(mid)] = 0
                            continue

                        self._save_motor_state_if_needed(mid, m)
                        sign = int(FACE_TRACK_MOTOR_SIGN.get(mid, 1))
                        direction = "cw" if (cmd * sign) > 0 else "ccw"
                        kp = FACE_TRACK_KP_SPS_PER_PX * pan_speed * max(0.35, abs(mix_x) + abs(mix_y))
                        is_primary = (mid == 3)
                        max_ratio = 1.0 if is_primary else FACE_TRACK_ASSIST_MAX_RATIO
                        min_ratio = 1.0 if is_primary else FACE_TRACK_ASSIST_MIN_RATIO
                        max_sps = min(
                            FACE_TRACK_MAX_SPS_HARD,
                            int(FACE_TRACK_MAX_SPS * pan_speed * max_ratio),
                        )
                        min_sps = int(max(60, FACE_TRACK_MIN_SPS * min_ratio))
                        motor_sps = int(min(
                            max_sps,
                            max(min_sps, min_sps + abs(cmd) * kp),
                        ))
                        if not self._motor_active.get(mid, False):
                            m.set_direction(direction)
                            m.set_speed(motor_sps)
                            m.enable()
                            m.run_pulses()
                            self._motor_active[mid] = True
                            self._motor_dir[mid] = direction
                            self._motor_last_sps[mid] = motor_sps
                        else:
                            if direction != self._motor_dir.get(mid):
                                m.set_direction(direction)
                                self._motor_dir[mid] = direction
                            if abs(motor_sps - self._motor_last_sps.get(mid, 0)) >= FACE_TRACK_SPS_STEP:
                                m.set_speed(motor_sps)
                                self._motor_last_sps[mid] = motor_sps
                            else:
                                motor_sps = self._motor_last_sps.get(mid, motor_sps)
                        motor_sps_map[str(mid)] = int(motor_sps)
                    except Exception as exc:
                        motor_sps_map[str(mid)] = 0
                        self._set_state(error=str(exc))

                # Tilt with Axis 5 servo.
                servo_angle = servo_angles.get(FACE_TRACK_SERVO_ID, 0.0)
                if FACE_TRACK_SERVO_ID in servos and servo_enabled.get(FACE_TRACK_SERVO_ID, False):
                    # Low-pass the vertical error to reject per-frame jitter.
                    ya = max(0.0, min(1.0, FACE_TRACK_Y_FILTER_ALPHA))
                    ey_tgt = float(ey) - float(FACE_TRACK_Y_TARGET_OFFSET_PX)
                    self._tilt_err_f = (1.0 - ya) * self._tilt_err_f + ya * ey_tgt
                    eyf = self._tilt_err_f

                    is_up = (eyf < 0.0)
                    if is_up:
                        stop_zone = max(0, int(FACE_TRACK_Y_UP_DEADZONE_PX))
                        start_zone = max(stop_zone + 1, int(FACE_TRACK_Y_UP_START_PX))
                    else:
                        stop_zone = max(0, int(FACE_TRACK_Y_DEADZONE_PX))
                        start_zone = max(stop_zone + 1, int(FACE_TRACK_Y_START_PX))
                    if self._tilt_active:
                        tilt_should_move = abs(eyf) > stop_zone
                    else:
                        tilt_should_move = abs(eyf) >= start_zone

                    if tilt_should_move:
                        self._tilt_active = True
                        # Fixed proportional gain keeps the loop stable; the
                        # user tilt-speed only raises the per-loop slew cap so a
                        # far face is reached faster WITHOUT overshooting near
                        # centre (overshoot was the up/down oscillation).
                        kp = FACE_TRACK_TILT_KP_DEG_PER_PX * (FACE_TRACK_TILT_UP_GAIN if is_up else 1.0)
                        delta = -FACE_TRACK_TILT_SIGN * eyf * kp
                        tilt_cap = FACE_TRACK_TILT_MAX_STEP_DEG * tilt_speed * (
                            FACE_TRACK_TILT_UP_CAP_SCALE if is_up else 1.0
                        )
                        if delta > tilt_cap:
                            delta = tilt_cap
                        elif delta < -tilt_cap:
                            delta = -tilt_cap
                        # Clamp to the user Hard Limit so tilt can never crash
                        # the servo into its mechanical end-stops.
                        lo = min(tilt_min, tilt_max)
                        hi = max(tilt_min, tilt_max)
                        target = max(lo, min(hi, servo_angle + delta))
                        with servo_lock:
                            _cancel_servo_detach(FACE_TRACK_SERVO_ID)
                            _set_servo_angle_hw(FACE_TRACK_SERVO_ID, target)
                            servo_angles[FACE_TRACK_SERVO_ID] = target
                            servo_angle = target
                    else:
                        self._tilt_active = False

                self._set_state(
                    live=True,
                    has_face=True,
                    face_center=[cx, cy],
                    error_px=[ex, ey],
                    motor_sps=max([int(v) for v in motor_sps_map.values()] or [0]),
                    motor_sps_map=motor_sps_map,
                    servo_angle=round(float(servo_angle), 2),
                    error=None,
                )
            else:
                if (time.time() - last_face_ts) > FACE_TRACK_LOST_HOLD_S:
                    # Keep tracker outputs prepared while face is temporarily
                    # lost so re-acquisition resumes immediately.
                    self._stop_outputs(restore_motor=False, mark_not_ready=False)
                self._set_state(
                    live=True,
                    has_face=False,
                    face_center=None,
                    error_px=[0, 0],
                    motor_sps=0,
                    motor_sps_map={str(mid): 0 for mid in FACE_TRACK_MOTOR_IDS},
                )

            time.sleep(FACE_TRACK_LOOP_DT)


face_tracker = FaceTracker(camera)


class EmotionRings:
    """Animate WS2812 rings according to detected emotion.

    Both 8-LED rings share one data line. On the Raspberry Pi 5 the data line is
    driven from the SPI peripheral (MOSI = GPIO10, physical pin 19); the legacy
    ws281x / pigpio DMA-PWM backends are kept as fallbacks for older boards.
    """

    UPDATE_DT = 0.06
    HOLD_LAST_S = 1.8

    def __init__(self, camera):
        self.camera = camera
        self.available = False
        self.backend = None
        self.error = None
        self._strip = None
        self._pi = None
        self._spi = None
        self._enabled = True
        self._running = False
        self._thread = None
        self._step = 0
        self._test_until = 0.0
        self._test_step = 0
        self._last_emotion = "neutral"
        self._last_seen_ts = 0.0
        self._rng_state = 0xC0FFEE
        spi_err = None
        if SPIDEV_AVAILABLE:
            try:
                self._spi = spidev.SpiDev()
                self._spi.open(RING_SPI_BUS, RING_SPI_DEV)
                self._spi.max_speed_hz = RING_SPI_HZ
                self._spi.mode = 0
                self.available = True
                self.backend = "spi"
                self._blackout()
                return
            except Exception as e:
                spi_err = e
                if self._spi is not None:
                    try:
                        self._spi.close()
                    except Exception:
                        pass
                self._spi = None

        ws_err = None
        if WS281X_AVAILABLE:
            try:
                self._strip = PixelStrip(
                    RING_LED_COUNT,
                    18,
                    800000,
                    10,
                    False,
                    RING_BRIGHTNESS,
                    0,
                )
                self._strip.begin()
                self.available = True
                self.backend = "ws281x"
                self._blackout()
                return
            except Exception as e:
                ws_err = e
                self._strip = None

        pig_err = None
        if PIGPIO_AVAILABLE:
            try:
                self._pi = pigpio.pi()
                if not self._pi.connected:
                    raise RuntimeError("pigpiod not reachable")
                self._pi.set_mode(18, pigpio.OUTPUT)
                self.available = True
                self.backend = "pigpio"
                self._blackout()
                return
            except Exception as e:
                pig_err = e
                if self._pi is not None:
                    try:
                        self._pi.stop()
                    except Exception:
                        pass
                self._pi = None

        self.error = "init failed: spi=%r ws281x=%r pigpio=%r" % (
            spi_err, ws_err, pig_err)

    @staticmethod
    def _clamp_u8(v):
        return max(0, min(255, int(round(v))))

    @classmethod
    def _scale(cls, rgb, factor):
        return (
            cls._clamp_u8(rgb[0] * factor),
            cls._clamp_u8(rgb[1] * factor),
            cls._clamp_u8(rgb[2] * factor),
        )

    @staticmethod
    def _wheel(pos):
        pos = int(pos) % 256
        if pos < 85:
            return (pos * 3, 255 - pos * 3, 0)
        if pos < 170:
            pos -= 85
            return (255 - pos * 3, 0, pos * 3)
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

    def _rand_u8(self):
        # Lightweight deterministic PRNG for flicker/spark animation.
        self._rng_state = (1103515245 * self._rng_state + 12345) & 0x7FFFFFFF
        return self._rng_state & 0xFF

    def _spi_show(self, colors):
        """Clock one WS2812 frame out of SPI MOSI (GRB order).

        Each WS2812 bit becomes 3 SPI bits at ~2.4 MHz: a '1' is 0b110 (long
        high), a '0' is 0b100 (short high). 8 data bits -> 24 SPI bits -> 3
        bytes, so encoding always lands on byte boundaries.
        """
        if self._spi is None:
            raise RuntimeError("spi backend unavailable")
        scale = RING_BRIGHTNESS / 255.0
        bits = []
        for (r, g, b) in colors:
            gr = int(g * scale)
            rd = int(r * scale)
            bl = int(b * scale)
            for byte in (gr, rd, bl):
                for i in range(7, -1, -1):
                    if (byte >> i) & 1:
                        bits.extend((1, 1, 0))
                    else:
                        bits.extend((1, 0, 0))
        out = bytearray()
        for i in range(0, len(bits), 8):
            chunk = bits[i:i + 8]
            while len(chunk) < 8:
                chunk.append(0)
            byte = 0
            for bit in chunk:
                byte = (byte << 1) | bit
            out.append(byte)
        # Trailing zero bytes hold the line low for the >50 us reset latch.
        out.extend(b"\x00" * 40)
        self._spi.writebytes2(out)

    def _pigpio_show(self, colors):
        """Send one WS2812 frame through pigpio waves (GRB, 800kHz)."""
        if self._pi is None:
            raise RuntimeError("pigpio backend unavailable")
        pin_mask = 1 << 18
        # Approximate WS2812 timings in microseconds.
        t0h, t0l = 1, 1
        t1h, t1l = 1, 1
        pulses = []
        for (r, g, b) in colors:
            for byte in (g, r, b):
                for bit in range(7, -1, -1):
                    if byte & (1 << bit):
                        pulses.append(pigpio.pulse(pin_mask, 0, t1h))
                        pulses.append(pigpio.pulse(0, pin_mask, t1l))
                    else:
                        pulses.append(pigpio.pulse(pin_mask, 0, t0h))
                        pulses.append(pigpio.pulse(0, pin_mask, t0l))
        # Reset latch.
        pulses.append(pigpio.pulse(0, pin_mask, 300))

        self._pi.wave_clear()
        self._pi.wave_add_generic(pulses)
        wid = self._pi.wave_create()
        if wid < 0:
            raise RuntimeError("pigpio wave_create failed")
        self._pi.wave_send_once(wid)
        while self._pi.wave_tx_busy():
            time.sleep(0.001)
        self._pi.wave_delete(wid)

    def _apply(self, ring1_colors, ring2_colors):
        if not self.available:
            return
        if self.backend == "spi":
            self._spi_show(ring1_colors)
            return
        if self.backend == "ws281x":
            if not self._strip:
                return
            for i in range(RING_LED_COUNT):
                c1 = ring1_colors[i]
                self._strip.setPixelColor(i, Color(c1[0], c1[1], c1[2]))
            self._strip.show()
            return
        if self.backend == "pigpio":
            self._pigpio_show(ring1_colors)

    def _blackout(self):
        off = [(0, 0, 0)] * RING_LED_COUNT
        self._apply(off, off)

    def _render(self, emotion, step):
        """Return (ring1_colors, ring2_colors) for the active emotion."""
        t = step
        if emotion == "happy":
            # Fast rainbow swirl with opposite spin on the second ring.
            a = [self._wheel(i * 32 + t * 8) for i in range(RING_LED_COUNT)]
            b = [self._wheel((RING_LED_COUNT - 1 - i) * 32 + t * 8)
                 for i in range(RING_LED_COUNT)]
            return a, b

        if emotion == "sad":
            # Slow blue breathing.
            breath = (1.0 + np.sin(t / 10.0)) * 0.5 if np is not None else 0.5
            base = self._scale((30, 70, 255), 0.12 + 0.55 * breath)
            cols = [base] * RING_LED_COUNT
            return cols, cols

        if emotion == "angry":
            # Red pulse with occasional hot-orange sparks.
            pulse = (1.0 + np.sin(t / 2.2)) * 0.5 if np is not None else 0.5
            base = self._scale((255, 18, 0), 0.45 + 0.5 * pulse)
            a = [base] * RING_LED_COUNT
            b = [base] * RING_LED_COUNT
            if t % 3 == 0:
                idx = (t // 3) % RING_LED_COUNT
                spark = (255, 100, 0)
                a[idx] = spark
                b[(RING_LED_COUNT - 1 - idx)] = spark
            return a, b

        if emotion == "fear":
            # Uneasy violet flicker.
            a, b = [], []
            for i in range(RING_LED_COUNT):
                f1 = 0.12 + (self._rand_u8() / 255.0) * 0.55
                f2 = 0.12 + (self._rand_u8() / 255.0) * 0.55
                a.append(self._scale((130, 0, 255), f1))
                b.append(self._scale((100, 0, 220), f2))
            return a, b

        if emotion == "surprise":
            # White flash + cyan orbit.
            flash = (t % 6) in (0, 1)
            if flash:
                bright = [(255, 255, 255)] * RING_LED_COUNT
                return bright, bright
            idx = (t // 2) % RING_LED_COUNT
            a = [(15, 30, 40)] * RING_LED_COUNT
            b = [(15, 30, 40)] * RING_LED_COUNT
            a[idx] = (0, 220, 255)
            b[(RING_LED_COUNT - 1 - idx)] = (0, 220, 255)
            return a, b

        if emotion == "neutral":
            # Calm mint ring with gentle rotating highlight.
            idx = (t // 3) % RING_LED_COUNT
            a = [(8, 40, 32)] * RING_LED_COUNT
            b = [(8, 40, 32)] * RING_LED_COUNT
            a[idx] = (80, 180, 150)
            b[(RING_LED_COUNT - 1 - idx)] = (80, 180, 150)
            return a, b

        # No face / unknown: low-power amber heartbeat.
        beat = 0.10 + (0.18 if (t % 24) in (0, 1, 2) else 0.0)
        dim = self._scale((255, 120, 20), beat)
        cols = [dim] * RING_LED_COUNT
        return cols, cols

    def _pick_emotion(self):
        data = self.camera.peek_result()
        now = time.time()
        if data.get("emo_enabled") and data.get("live") and data.get("face"):
            dom = data.get("dominant")
            if dom in UI_EMOTIONS:
                self._last_emotion = dom
                self._last_seen_ts = now
                return dom
        if now - self._last_seen_ts < self.HOLD_LAST_S:
            return self._last_emotion
        return None

    def ensure_started(self):
        if not self.available:
            return False
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def set_enabled(self, on):
        self._enabled = bool(on)
        if not self._enabled:
            self._blackout()
        return self._enabled

    def start_test(self, seconds=4.0):
        """Run a self-test color sweep for a few seconds, then resume emotions."""
        if not self.available:
            return False
        self._test_until = time.monotonic() + max(0.5, float(seconds))
        self._test_step = 0
        return True

    def _render_test(self, step):
        """Diagnostic pattern: solid R, G, B, white, then a single chasing dot."""
        phase = (step // 12) % 5
        if phase == 0:
            solid = (255, 0, 0)
        elif phase == 1:
            solid = (0, 255, 0)
        elif phase == 2:
            solid = (0, 0, 255)
        elif phase == 3:
            solid = (255, 255, 255)
        else:
            solid = None
        if solid is not None:
            colors = [solid] * RING_LED_COUNT
        else:
            dot = step % RING_LED_COUNT
            colors = [(0, 0, 0)] * RING_LED_COUNT
            colors[dot] = (255, 160, 0)
        return colors, colors

    def status(self):
        data_pin = 10 if self.backend == "spi" else 18
        return {
            "available": self.available,
            "backend": self.backend,
            "enabled": self._enabled,
            "running": self._running,
            "pins": [data_pin],
            "count": RING_LED_COUNT,
            "emotion": self._last_emotion,
            "error": self.error,
        }

    def _run(self):
        while self._running:
            try:
                if time.monotonic() < self._test_until:
                    ring1, ring2 = self._render_test(self._test_step)
                    self._apply(ring1, ring2)
                    self._test_step += 1
                    time.sleep(self.UPDATE_DT)
                    continue
                if not self._enabled:
                    time.sleep(0.2)
                    continue
                emotion = self._pick_emotion()
                ring1, ring2 = self._render(emotion, self._step)
                self._apply(ring1, ring2)
                self._step += 1
            except Exception as e:
                # Keep the service alive even if LED IO glitches.
                self.error = "runtime failed: %r" % (e,)
            time.sleep(self.UPDATE_DT)


rings = EmotionRings(camera)
rings.ensure_started()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/emotion")
def emotion_page():
    return render_template("emotion.html")


@app.route("/emotion/latest")
def emotion_latest():
    """Latest face / emotion result from the live camera pipeline.

    Returns ``live: true`` with real scores once the camera + inference thread
    are running, otherwise ``live: false`` so the UI falls back to its built-in
    simulation. Shape::

        {
          "live": true,
          "face": 1,
          "box": [x, y, w, h],            # in the 640x480 frame
          "scores": {"happy": .., "neutral": .., "surprise": ..,
                     "sad": .., "angry": .., "fear": ..},
          "dominant": "happy",
          "infer_ms": 12.3,
          "fps": 14.8
        }
    """
    return jsonify(camera.latest_result())


@app.route("/camera/stream")
def camera_stream():
    """MJPEG stream of the annotated camera feed."""
    if not camera.available:
        return jsonify({"error": "camera unavailable"}), 503
    return Response(
        camera.frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/camera/snapshot")
def camera_snapshot():
    """Single annotated JPEG frame from the camera."""
    if not camera.available:
        return jsonify({"error": "camera unavailable"}), 503
    jpg = camera.snapshot()
    if jpg is None:
        return jsonify({"error": "no frame"}), 503
    return Response(jpg, mimetype="image/jpeg")


@app.route("/emotion/enable", methods=["POST"])
def emotion_enable():
    """Turn emotion detection on or off ({"on": true|false})."""
    data = request.get_json(silent=True) or {}
    on = bool(data.get("on", True))
    enabled = camera.set_emotion(on)
    return jsonify({"enabled": enabled})


@app.route("/emotion/rings", methods=["GET", "POST"])
def emotion_rings():
    """Get or set emotion-ring animation state."""
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        on = _as_bool(data.get("on", True), default=True)
        enabled = rings.set_enabled(on)
        st = rings.status()
        st["enabled"] = enabled
        return jsonify(st)
    return jsonify(rings.status())


@app.route("/emotion/rings/test", methods=["POST"])
def emotion_rings_test():
    """Fire a short color self-test sweep on the rings."""
    data = request.get_json(silent=True) or {}
    seconds = float(data.get("seconds", 4.0))
    started = rings.start_test(seconds)
    return jsonify({"started": started, "available": rings.available})


@app.route("/face_tracking/status")
def face_tracking_status():
    return jsonify(face_tracker.status())


@app.route("/face_tracking/enable", methods=["POST"])
def face_tracking_enable():
    data = request.get_json(silent=True)
    on_raw = None
    if isinstance(data, dict):
        on_raw = data.get("on")
    if on_raw is None:
        on_raw = request.values.get("on")
    on = _as_bool(on_raw, default=False)
    if on:
        face_tracker.start()
    else:
        face_tracker.stop()
    st = face_tracker.status()
    return jsonify({"enabled": st.get("enabled", False), "status": st})


@app.route("/face_tracking/speed", methods=["POST"])
def face_tracking_speed():
    data = request.get_json(silent=True)
    pan = tilt = None
    if isinstance(data, dict):
        pan = data.get("pan")
        tilt = data.get("tilt")
    if pan is None:
        pan = request.values.get("pan")
    if tilt is None:
        tilt = request.values.get("tilt")
    result = face_tracker.set_speed(pan=pan, tilt=tilt)
    return jsonify({"status": "ok", **result})


@app.route("/face_tracking/tilt_limits", methods=["POST"])
def face_tracking_tilt_limits():
    data = request.get_json(silent=True)
    lo = hi = None
    if isinstance(data, dict):
        lo = data.get("min")
        hi = data.get("max")
    if lo is None:
        lo = request.values.get("min")
    if hi is None:
        hi = request.values.get("max")
    result = face_tracker.set_tilt_limits(min_deg=lo, max_deg=hi)
    return jsonify({"status": "ok", **result})


@app.route("/joint_states")
def joint_states():
    return jsonify(twin_poller.snapshot())


@app.route("/twin/enable", methods=["POST"])
def twin_enable():
    data = request.get_json(silent=True)
    on_raw = data.get("on") if isinstance(data, dict) else None
    if on_raw is None:
        on_raw = request.values.get("on")
    on = _as_bool(on_raw, default=False)
    if on:
        twin_poller.start()
    else:
        twin_poller.stop()
    return jsonify(twin_poller.snapshot())


@app.route("/twin/zero", methods=["POST"])
def twin_zero():
    return jsonify({"status": "ok", **twin_poller.set_zero()})


@app.route("/twin/config")
def twin_config():
    """Travel limits (radians) and which twin joints command hardware."""
    commandable = sorted(list(TWIN_COMMAND_STEPPERS) + list(TWIN_COMMAND_SERVOS))
    return jsonify({
        "limits": {f"joint{ji}": v for ji, v in TWIN_JOINT_LIMITS.items()},
        "commandable": [f"joint{ji}" for ji in commandable],
    })


@app.route("/twin/limits", methods=["POST"])
def twin_set_limits():
    data = request.get_json(silent=True) or {}
    lims = data.get("limits") or {}
    for key, val in lims.items():
        try:
            ji = int(str(key).replace("joint", ""))
            lo, hi = float(val[0]), float(val[1])
            if lo > hi:
                lo, hi = hi, lo
            TWIN_JOINT_LIMITS[ji] = [lo, hi]
        except (ValueError, TypeError, IndexError):
            continue
    return jsonify({"limits": {f"joint{ji}": v
                               for ji, v in TWIN_JOINT_LIMITS.items()}})


@app.route("/twin/move", methods=["POST"])
def twin_move_route():
    data = request.get_json(silent=True) or {}
    angles = data.get("angles") or {}
    speed = _clamp(float(data.get("speed", 0.4) or 0.4), 0.05, 1.0)
    started, msg = twin_move(angles, speed)
    return jsonify({"started": started, "message": msg, **_twin_move_status()})


@app.route("/twin/move_status")
def twin_move_status_route():
    return jsonify(_twin_move_status())


@app.route("/twin/stop", methods=["POST"])
def twin_stop_route():
    arm._stop.set()
    with _twin_move_lock:
        _twin_move_state.update(moving=False, message="stopped")
    for m in motors.values():
        try:
            m.hold()
        except Exception:
            pass
    return jsonify(_twin_move_status())


@app.route("/led/on", methods=["POST"])
def led_on():
    write_led(LED_TRIGGER, "none")
    write_led(LED_BRIGHTNESS, "0")
    return jsonify({"status": "on"})


@app.route("/led/off", methods=["POST"])
def led_off():
    write_led(LED_TRIGGER, "none")
    write_led(LED_BRIGHTNESS, "1")
    return jsonify({"status": "off"})


@app.route("/led/status")
def led_status():
    brightness = read_brightness()
    return jsonify({"status": "on" if brightness == "0" else "off"})


@app.route("/system/temp")
def system_temp():
    """Onboard SoC temperature in degrees Celsius."""
    return jsonify({"temp_c": read_cpu_temp_c()})


@app.route("/system/health")
def system_health():
    """Aggregate Pi 5 health metrics (temp, CPU, memory, disk, power)."""
    return jsonify(read_health())


# -- Stepper motors ---------------------------------------------------------
@app.route("/motor/<int:mid>/enable", methods=["POST"])
def motor_enable(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    motor.enable()
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/disable", methods=["POST"])
def motor_disable(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    # Default to a soft stop (ramp down); pass {"soft": false} for an
    # immediate hard stop / emergency cut.
    soft = data.get("soft", True)
    motor.disable(soft=bool(soft))
    return jsonify(motor.status())


@app.route("/estop", methods=["POST"])
def estop():
    """Emergency stop: immediately hard-stop every motor and any playback."""
    face_tracker.stop()
    arm.stop_playback()
    for m in motors.values():
        m.disable(soft=False)
    return jsonify({"stopped": list(motors.keys())})


@app.route("/motor/<int:mid>/direction", methods=["POST"])
def motor_direction(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    direction = data.get("direction", "cw")
    try:
        motor.set_direction(direction)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/jog", methods=["POST"])
def motor_jog(mid):
    """Momentary jog: move only while the button is held.

    Body: ``{"direction": "cw"|"ccw", "on": true|false}``. With ``on`` true the
    motor energizes (if needed) and emits step pulses in the given direction;
    with ``on`` false it stops emitting pulses but stays energized (holds
    position). The motor never moves from the Enable button — only from here.
    """
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    on = _as_bool(data.get("on", True), default=True)
    if on:
        # Head tracking can use multiple motors. Manual jog always takes
        # priority so the operator never fights the tracker.
        if mid in FACE_TRACK_MOTOR_IDS:
            face_tracker.stop()
        direction = data.get("direction")
        if direction is not None:
            try:
                motor.set_direction(direction)
            except ValueError as exc:
                return jsonify({"error": str(exc)}), 400
        motor.enable()       # energize + start worker (held)
        motor.run_pulses()   # release the hold -> move while pressed
    else:
        motor.hold()         # stop moving, stay energized
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/speed", methods=["POST"])
def motor_speed(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    try:
        motor.set_speed(data.get("speed", DEFAULT_SPEED))
    except (TypeError, ValueError):
        return jsonify({"error": "speed must be an integer"}), 400
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/rpm", methods=["POST"])
def motor_rpm(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    try:
        motor.set_rpm(data.get("rpm", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "rpm must be a number"}), 400
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/geometry", methods=["POST"])
def motor_geometry(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    try:
        motor.set_geometry(
            full_steps_per_rev=data.get("full_steps_per_rev"),
            microstepping=data.get("microstepping"),
        )
    except (TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/accel", methods=["POST"])
def motor_accel(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    data = request.get_json(silent=True) or {}
    try:
        motor.set_accel(data.get("accel", DEFAULT_ACCEL))
    except (TypeError, ValueError):
        return jsonify({"error": "accel must be an integer"}), 400
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/status")
def motor_status(mid):
    motor = get_motor(mid)
    if motor is None:
        return jsonify({"error": "unknown motor"}), 404
    return jsonify(motor.status())


@app.route("/motor/<int:mid>/encoder")
def motor_encoder(mid):
    enc = get_encoder(mid)
    if enc is None:
        return jsonify({"error": "unknown motor"}), 404
    return jsonify(enc.read())


# -- Servo motors (axis 5 & 6) -------------------------------------------------
@app.route("/servo/<int:sid>/angle", methods=["GET", "POST"])
def servo_angle(sid):
    """Get or set a servo's angle for 270deg servos (-135 to 135 degrees)."""
    if sid not in servos:
        return jsonify({"error": f"servo {sid} not available"}), 404
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        angle = float(data.get("angle", 0))
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        with servo_lock:
            if not servo_enabled.get(sid, True):
                return jsonify({
                    "error": f"servo {sid} is OFF",
                    "servo": sid,
                    "enabled": False,
                }), 409
            prev = servo_angles.get(sid)
            # Ignore tiny request jitter from UI/network noise.
            if prev is None or abs(prev - angle) >= 0.5:
                if sid == 5 and prev is not None:
                    _set_servo_angle_smooth(sid, prev, angle)
                else:
                    _set_servo_angle_hw(sid, angle)
                servo_angles[sid] = angle
            if servo_hold.get(sid, True):
                _cancel_servo_detach(sid)
            else:
                _schedule_servo_detach(sid)
            return jsonify({
                "servo": sid,
                "angle": angle,
                "enabled": servo_enabled.get(sid, True),
                "hold": servo_hold.get(sid, True),
                "detach_delay": servo_detach_delay.get(sid, 0.25),
            })
    angle = servo_angles.get(sid, 0.0)
    return jsonify({
        "servo": sid,
        "angle": angle,
        "enabled": servo_enabled.get(sid, True),
        "hold": servo_hold.get(sid, True),
        "detach_delay": servo_detach_delay.get(sid, 0.25),
    })


@app.route("/servo/<int:sid>/power", methods=["POST"])
def servo_power(sid):
    """Turn servo PWM output ON/OFF (OFF detaches signal)."""
    if sid not in servos:
        return jsonify({"error": f"servo {sid} not available"}), 404
    data = request.get_json(silent=True) or {}
    on = _as_bool(data.get("on", True), default=True)
    with servo_lock:
        servo_enabled[sid] = on
        if on:
            angle = servo_angles.get(sid, 0.0)
            prev = servo_angles.get(sid, angle)
            if sid == 5 and prev is not None:
                _set_servo_angle_smooth(sid, prev, angle)
            else:
                _set_servo_angle_hw(sid, angle)
            if servo_hold.get(sid, True):
                _cancel_servo_detach(sid)
            else:
                _schedule_servo_detach(sid)
        else:
            _cancel_servo_detach(sid)
            pwm = servos.get(sid)
            if pwm is not None:
                try:
                    pwm.change_duty_cycle(0)
                except Exception:
                    pass
    return jsonify({
        "servo": sid,
        "enabled": servo_enabled.get(sid, True),
        "hold": servo_hold.get(sid, True),
        "angle": servo_angles.get(sid, 0.0),
        "detach_delay": servo_detach_delay.get(sid, 0.25),
    })


@app.route("/servo/<int:sid>/hold", methods=["POST"])
def servo_hold_mode(sid):
    """Set hold mode. ON keeps torque at target; OFF detaches after moves."""
    if sid not in servos:
        return jsonify({"error": f"servo {sid} not available"}), 404
    data = request.get_json(silent=True) or {}
    on = bool(data.get("on", True))
    with servo_lock:
        servo_hold[sid] = on
        if not servo_enabled.get(sid, True):
            return jsonify({
                "servo": sid,
                "enabled": False,
                "hold": servo_hold.get(sid, True),
                "angle": servo_angles.get(sid, 0.0),
                "detach_delay": servo_detach_delay.get(sid, 0.25),
            })
        if on:
            _cancel_servo_detach(sid)
            angle = servo_angles.get(sid, 0.0)
            prev = servo_angles.get(sid, angle)
            if sid == 5 and prev is not None:
                _set_servo_angle_smooth(sid, prev, angle)
            else:
                _set_servo_angle_hw(sid, angle)
        else:
            _schedule_servo_detach(sid)
    return jsonify({
        "servo": sid,
        "enabled": servo_enabled.get(sid, True),
        "hold": servo_hold.get(sid, True),
        "angle": servo_angles.get(sid, 0.0),
        "detach_delay": servo_detach_delay.get(sid, 0.25),
    })


@app.route("/servo/<int:sid>/detach_delay", methods=["POST"])
def servo_detach_delay_set(sid):
    """Set idle detach delay (seconds) used when hold mode is OFF."""
    if sid not in servos:
        return jsonify({"error": f"servo {sid} not available"}), 404
    data = request.get_json(silent=True) or {}
    delay = float(data.get("seconds", 0.25))
    delay = max(0.05, min(2.0, delay))
    with servo_lock:
        servo_detach_delay[sid] = delay
        if servo_enabled.get(sid, True) and not servo_hold.get(sid, True):
            _schedule_servo_detach(sid)
    return jsonify({
        "servo": sid,
        "enabled": servo_enabled.get(sid, True),
        "hold": servo_hold.get(sid, True),
        "angle": servo_angles.get(sid, 0.0),
        "detach_delay": servo_detach_delay.get(sid, 0.25),
    })


@app.route("/servo/status")
def servo_status():
    """Return status of all servos."""
    status = {}
    for sid, servo in servos.items():
        angle = servo_angles.get(sid, 0.0)
        status[sid] = {
            "available": True,
            "angle": angle if angle is not None else 0.0,
            "pin": [SERVO5_PIN if sid == 5 else SERVO6_PIN],
            "enabled": servo_enabled.get(sid, True),
            "hold": servo_hold.get(sid, True),
            "detach_delay": servo_detach_delay.get(sid, 0.25),
            "range": [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE],
        }
    return jsonify(status)


# -- Robot arm: teach & playback -------------------------------------------
@app.route("/arm/capture", methods=["POST"])
def arm_capture():
    return jsonify(arm.capture())


@app.route("/arm/freedrive", methods=["POST"])
def arm_freedrive():
    data = request.get_json(silent=True) or {}
    arm.free_drive(bool(data.get("on", False)))
    return jsonify(arm.get_state())


@app.route("/arm/play", methods=["POST"])
def arm_play():
    data = request.get_json(silent=True) or {}
    waypoints = data.get("waypoints") or []
    if not waypoints:
        return jsonify({"error": "no waypoints"}), 400
    try:
        speed = float(data.get("speed", 1.0))
        loops = int(data.get("loops", 1))
        dwell = float(data.get("dwell", 0.5))
    except (TypeError, ValueError):
        return jsonify({"error": "invalid play parameters"}), 400
    if not arm.start_playback(waypoints, speed, loops, dwell):
        return jsonify({"error": "already playing"}), 409
    return jsonify(arm.get_state())


@app.route("/arm/stop", methods=["POST"])
def arm_stop():
    arm.stop_playback()
    return jsonify(arm.get_state())


@app.route("/arm/status")
def arm_status():
    return jsonify(arm.get_state())


@app.route("/programs", methods=["GET"])
def programs_list():
    return jsonify({"programs": arm.list_programs()})


@app.route("/programs/<name>", methods=["GET"])
def program_get(name):
    try:
        return jsonify(arm.load_program(name))
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except FileNotFoundError:
        return jsonify({"error": "not found"}), 404


@app.route("/programs/<name>", methods=["POST"])
def program_save(name):
    data = request.get_json(silent=True) or {}
    try:
        arm.save_program(name, data)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify({"saved": name})


@app.route("/programs/<name>", methods=["DELETE"])
def program_delete(name):
    try:
        arm.delete_program(name)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify({"deleted": name})


if __name__ == "__main__":
    try:
        # threaded=True so a slow encoder read on the shared UART bus never
        # blocks motor enable/disable/status requests.
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        arm.stop_playback()
        for _m in motors.values():
            _m.disable(soft=False)
