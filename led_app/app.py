import os
import json
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
        AngularServo,
    )

    GPIO_AVAILABLE = True
except Exception:  # gpiozero not installed / not running on a Pi
    DigitalOutputDevice = None
    PWMOutputDevice = None
    DigitalInputDevice = None
    AngularServo = None
    GPIO_AVAILABLE = False

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
#   Motor 4:  EN -> GPIO 0    STP -> GPIO 5    DIR -> GPIO 6
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

# Motor 3 end-stop limit switches. Both travel-limit switches share a SINGLE
# GPIO line (GPIO 26): each is wired to 3.3V (the Pi GPIO is 3.3V only — never
# 5V), so a pressed switch drives the pin HIGH and an internal pull-down holds
# it LOW when released. Only one end stop can be reached at a time, so the
# motor's current travel direction tells us which limit was hit — no need for a
# separate pin per switch. When the line trips the motor stops immediately and
# refuses to drive further that way; jogging the opposite direction backs off.
M3_LIMIT_PIN = 26
M4_LIMIT_PIN = 19

# Servo motors for axis 5 & 6 (standard RC servos with 50 Hz PWM).
# GPIO 12/13 are hardware PWM (PWM0 ch0/ch1) on the Pi 5.
SERVO5_PIN = 12  # physical pin 32
SERVO6_PIN = 13  # physical pin 33
SERVO_MIN_ANGLE = -135
SERVO_MAX_ANGLE = 135
# DX-227 (270deg) generally accepts a wider pulse span than hobby 180deg servos.
SERVO_MIN_PULSE = 0.0005
SERVO_MAX_PULSE = 0.0025
SERVO_FRAME_WIDTH = 0.02

# Driver enable pin is active-LOW: drive LOW to energize the coils.
EN_ACTIVE_LOW = True

# Speed limits in steps per second. The hardware-timed PWM step generator can
# drive well past the old 2000 cap; 6000 sps = ~112 motor RPM at 1/16 stepping.
MIN_SPEED = 1
MAX_SPEED = 6000
DEFAULT_SPEED = 400

# Mechanical / driver geometry used to convert between steps/sec and RPM.
#   full_steps_per_rev: motor's native step count (1.8 deg NEMA 17 -> 200)
#   microstepping:      the SERVO42C microstep setting (e.g. 1, 16, 32, 256)
DEFAULT_FULL_STEPS_PER_REV = 200
DEFAULT_MICROSTEPPING = 16

# Acceleration ramp in steps per second^2. The running speed eases toward the
# target instead of jumping, which avoids stalling/skipping at high speeds.
MIN_ACCEL = 100
MAX_ACCEL = 50000
DEFAULT_ACCEL = 4000

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


class StepperMotor:
    """Generates step pulses on a background thread while the motor is enabled."""

    def __init__(self, en_pin, stp_pin, dir_pin, gear_ratio=1.0,
                 limit_pin=None):
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
        # End-stop state. A single shared switch line can't say WHICH end was
        # hit, so we latch the travel direction at the moment it trips; that
        # direction stays blocked until the switch releases, while the opposite
        # direction is allowed so the joint can back off.
        self.limit_stop = False
        self._blocked_dir = None
        self._stop = threading.Event()
        self._soft_stop = threading.Event()
        # When set, the worker keeps the coils energized but emits no pulses
        # (used to HOLD a position during closed-loop moves / playback).
        self._pause = threading.Event()
        self._thread = None

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
            # Optional shared end-stop switch: pull-down so idle reads LOW and a
            # press (3.3V) reads HIGH (is_active True). Small debounce.
            self._limit = (
                DigitalInputDevice(limit_pin, pull_up=False, bounce_time=0.005)
                if limit_pin is not None else None
            )
        else:
            self._en = self._step = self._dir = None
            self._limit = None

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

            # End-stop (single shared switch line). When the line is pressed we
            # latch the current travel direction as "blocked" (only one end stop
            # is reachable at a time, so the direction we're moving identifies
            # which limit was hit). That direction stays blocked until the line
            # releases; the opposite direction is allowed so the joint can back
            # off the switch.
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
            elif self.current_speed > target:
                self.current_speed = max(0.0, self.current_speed - step_accel)

            # A soft-stop fully ramps down, then exits the worker (de-energize).
            if soft and self.current_speed <= 0.0:
                break

            if self.current_speed <= 0.0:
                # Fully stopped: keep the coils energized but emit no pulses
                # (position hold). The smooth ramp-down already happened above.
                if self._step is not None:
                    self._step.value = 0.0
            else:
                # Moving or decelerating: emit pulses at the live ramped speed.
                speed = max(self.current_speed, float(MIN_SPEED))
                if self._step is not None:
                    self._step.frequency = max(1, int(round(speed)))
                    self._step.value = 0.5  # 50% duty -> emit step pulses
            time.sleep(RAMP_DT)

        # Worker is exiting (soft ramp-down completed). Stop pulses, de-energize
        # and clear state so the motor is fully stopped without blocking.
        self.current_speed = 0.0
        self.limit_stop = False
        self._blocked_dir = None
        if self._step is not None:
            self._step.value = 0.0  # stop emitting pulses
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

    def _limit_state(self):
        """End-stop info, or None if this motor has no limit switch."""
        if self._limit is None:
            return None
        return {"pressed": self._limit_pressed(), "blocked_dir": self._blocked_dir}

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
    2: StepperMotor(EN2_PIN, STP2_PIN, DIR2_PIN, MOTOR2_GEAR_RATIO),
    3: StepperMotor(
        EN3_PIN, STP3_PIN, DIR3_PIN, MOTOR3_GEAR_RATIO, limit_pin=M3_LIMIT_PIN,
    ),
    4: StepperMotor(
        EN4_PIN, STP4_PIN, DIR4_PIN, MOTOR4_GEAR_RATIO, limit_pin=M4_LIMIT_PIN,
    ),
}

# Standard RC servos for axis 5 & 6 using calibrated 270deg settings.
servos = {}
servo_angles = {}
if GPIO_AVAILABLE and AngularServo is not None:
    try:
        servos[5] = AngularServo(
            SERVO5_PIN,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=SERVO_MIN_PULSE,
            max_pulse_width=SERVO_MAX_PULSE,
            frame_width=SERVO_FRAME_WIDTH,
            initial_angle=0.0,
        )
        servos[6] = AngularServo(
            SERVO6_PIN,
            min_angle=SERVO_MIN_ANGLE,
            max_angle=SERVO_MAX_ANGLE,
            min_pulse_width=SERVO_MIN_PULSE,
            max_pulse_width=SERVO_MAX_PULSE,
            frame_width=SERVO_FRAME_WIDTH,
            initial_angle=0.0,
        )
        servo_angles[5] = 0.0
        servo_angles[6] = 0.0
    except Exception:
        pass  # Servo pins unavailable.


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
# Camera + on-device emotion detection
# ---------------------------------------------------------------------------
CAMERA_DEVICE = os.environ.get("MIRO_CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
_MODELS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FER_MODEL_PATH = os.path.join(_MODELS_DIR, "emotion-ferplus-8.onnx")
YUNET_MODEL_PATH = os.path.join(_MODELS_DIR, "face_detection_yunet_2023mar.onnx")

# Object detection (YOLOv4-tiny, COCO 80 classes, run via cv2.dnn DetectionModel).
YOLO_CFG_PATH = os.path.join(_MODELS_DIR, "yolov4-tiny.cfg")
YOLO_WEIGHTS_PATH = os.path.join(_MODELS_DIR, "yolov4-tiny.weights")
YOLO_NAMES_PATH = os.path.join(_MODELS_DIR, "coco.names")
OBJ_INPUT_SIZE = 224          # 224 vs 320 cuts inference ~50% with minor accuracy loss
OBJ_CONF_THRESHOLD = 0.45
OBJ_NMS_THRESHOLD = 0.4

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
        self.obj_available = (CV2_AVAILABLE and os.path.exists(YOLO_WEIGHTS_PATH)
                              and os.path.exists(YOLO_CFG_PATH))
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
        # object detection
        self._frame = None            # latest raw frame for the object thread
        self._obj_model = None
        self._obj_classes = []
        self._objects = []
        self._obj_enabled = False
        self._obj_thread = None
        self._obj_ms = 0.0
        self._emo_enabled = True    # emotion detection on by default

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

    # -- object detection ------------------------------------------------
    def _load_object_model(self):
        if self._obj_model is not None:
            return
        if not (os.path.exists(YOLO_WEIGHTS_PATH) and os.path.exists(YOLO_CFG_PATH)):
            return
        net = cv2.dnn.readNetFromDarknet(YOLO_CFG_PATH, YOLO_WEIGHTS_PATH)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        model = cv2.dnn.DetectionModel(net)
        model.setInputParams(size=(OBJ_INPUT_SIZE, OBJ_INPUT_SIZE),
                             scale=1.0 / 255.0, swapRB=True)
        self._obj_model = model
        if os.path.exists(YOLO_NAMES_PATH):
            with open(YOLO_NAMES_PATH) as f:
                self._obj_classes = [ln.strip() for ln in f if ln.strip()]

    def _detect_objects(self, frame):
        """Run YOLO on a frame, return a list of object dicts (sorted by conf)."""
        classes, confs, boxes = self._obj_model.detect(
            frame, OBJ_CONF_THRESHOLD, OBJ_NMS_THRESHOLD)
        classes = np.array(classes).reshape(-1)
        confs = np.array(confs).reshape(-1)
        boxes = np.array(boxes).reshape(-1, 4) if len(boxes) else np.empty((0, 4))
        w_f, h_f = float(self.width), float(self.height)
        objs = []
        for cid, conf, box in zip(classes, confs, boxes):
            x, y, w, h = (int(box[0]), int(box[1]), int(box[2]), int(box[3]))
            cx, cy = x + w / 2.0, y + h / 2.0
            cid = int(cid)
            label = self._obj_classes[cid] if cid < len(self._obj_classes) else str(cid)
            objs.append({
                "label": label,
                "confidence": round(float(conf), 3),
                "box": [x, y, w, h],
                "center": [int(round(cx)), int(round(cy))],
                # offset from frame centre, normalised to -1..1 (feed to motors)
                "offset": [round((cx / w_f) * 2.0 - 1.0, 3),
                           round((cy / h_f) * 2.0 - 1.0, 3)],
                "area": w * h,
            })
        objs.sort(key=lambda o: o["confidence"], reverse=True)
        return objs

    def _obj_run(self):
        try:
            self._load_object_model()
        except Exception:
            self._obj_model = None
        if self._obj_model is None:
            self._obj_enabled = False
            return
        while self._running and self._obj_enabled:
            with self._lock:
                frame = None if self._frame is None else self._frame.copy()
            if frame is None:
                time.sleep(0.05)
                continue
            t0 = time.time()
            try:
                objs = self._detect_objects(frame)
            except Exception:
                objs = []
            elapsed = time.time() - t0
            with self._lock:
                self._objects = objs
                self._obj_ms = elapsed * 1000.0
            # throttle to ~3 fps max (0.33 s budget) to keep CPU load manageable
            leftover = 0.33 - elapsed
            if leftover > 0:
                time.sleep(leftover)
        with self._lock:
            self._objects = []

    def set_emotion(self, on):
        """Enable/disable face + emotion classification."""
        self._emo_enabled = bool(on)
        return self._emo_enabled

    def set_objects(self, on):
        """Enable/disable the object-detection thread."""
        on = bool(on) and self.obj_available
        self._obj_enabled = on
        if on:
            self.ensure_started()
            with self._lock:
                alive = self._obj_thread is not None and self._obj_thread.is_alive()
                if not alive:
                    self._obj_thread = threading.Thread(
                        target=self._obj_run, daemon=True)
                    self._obj_thread.start()
        return self._obj_enabled

    def objects_result(self):
        self.ensure_started()
        with self._lock:
            objs = list(self._objects)
            enabled = self._obj_enabled
            running = self._running
            obj_ms = self._obj_ms
        return {
            "available": self.obj_available,
            "enabled": enabled,
            "live": running and enabled,
            "frame": [self.width, self.height],
            "count": len(objs),
            "objects": objs,
            "primary": objs[0] if objs else None,
            "infer_ms": round(obj_ms, 1),
        }

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
            if self._obj_enabled:
                with self._lock:
                    self._frame = frame.copy()
            t0 = time.time()
            emo_on = self._emo_enabled
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if emo_on else None
            box = None
            scores = None
            if emo_on:
                try:
                    box = self._detect_face(frame, gray)
                    if box is not None and self._emo_net is not None:
                        scores = self._classify(gray, box)
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

            if self._obj_enabled:
                with self._lock:
                    objs = list(self._objects)
                for o in objs:
                    ox, oy, ow, oh = o["box"]
                    cv2.rectangle(frame, (ox, oy), (ox + ow, oy + oh),
                                  (0, 170, 255), 2)
                    olabel = "%s %d%%" % (o["label"],
                                          round(o["confidence"] * 100))
                    cv2.rectangle(frame, (ox, max(0, oy - 20)),
                                  (ox + max(90, ow), oy), (20, 30, 40), -1)
                    cv2.putText(frame, olabel, (ox + 4, max(13, oy - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 170, 255), 2)

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
            # slow to 10 fps when object detection is running to share CPU
            target_fps = 10.0 if self._obj_enabled else 15.0
            frame_interval = 1.0 / target_fps
            dt = time.time() - t0
            if dt < frame_interval:
                time.sleep(frame_interval - dt)

        with self._lock:
            self._running = False
            self._jpeg = None
            self._result = self._empty_result()
            self._frame = None
            self._objects = []
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


@app.route("/objects/latest")
def objects_latest():
    """Latest object-detection result.

    Each object carries its pixel ``box``/``center`` plus a normalised
    ``offset`` (-1..1 from the frame centre) intended to drive the motors for
    physical tracking. ``primary`` is the highest-confidence object. Shape::

        {
          "available": true, "enabled": true, "live": true,
          "frame": [640, 480], "count": 2, "infer_ms": 180.0,
          "objects": [{"label": "person", "confidence": 0.91,
                       "box": [x, y, w, h], "center": [cx, cy],
                       "offset": [nx, ny], "area": 12345}, ...],
          "primary": { ...same as an object... }
        }
    """
    return jsonify(camera.objects_result())


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
        on = bool(data.get("on", True))
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


@app.route("/objects/enable", methods=["POST"])
def objects_enable():
    """Turn the object-detection thread on or off (``{"on": true|false}``)."""
    data = request.get_json(silent=True) or {}
    on = bool(data.get("on", True))
    enabled = camera.set_objects(on)
    return jsonify({"enabled": enabled, "available": camera.obj_available})


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
    on = bool(data.get("on", True))
    if on:
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
    servo = servos[sid]
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        angle = float(data.get("angle", 0))
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        prev = servo_angles.get(sid)
        # Ignore tiny request jitter from UI/network noise.
        if prev is None or abs(prev - angle) >= 0.5:
            servo.angle = angle
            servo_angles[sid] = angle
        return jsonify({"servo": sid, "angle": angle})
    angle = servo_angles.get(sid)
    if angle is None and getattr(servo, "angle", None) is not None:
        angle = float(servo.angle)
    return jsonify({"servo": sid, "angle": angle if angle is not None else 0.0})


@app.route("/servo/status")
def servo_status():
    """Return status of all servos."""
    status = {}
    for sid, servo in servos.items():
        angle = servo_angles.get(sid)
        if angle is None and getattr(servo, "angle", None) is not None:
            angle = float(servo.angle)
        status[sid] = {
            "available": True,
            "angle": angle if angle is not None else 0.0,
            "pin": [SERVO5_PIN if sid == 5 else SERVO6_PIN],
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
