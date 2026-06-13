import os
import json
import re
import socket
import threading
import time
import shutil
import subprocess

from flask import Flask, jsonify, render_template, request

try:
    from gpiozero import DigitalOutputDevice, PWMOutputDevice

    GPIO_AVAILABLE = True
except Exception:  # gpiozero not installed / not running on a Pi
    DigitalOutputDevice = None
    PWMOutputDevice = None
    GPIO_AVAILABLE = False

try:
    import serial  # pyserial

    SERIAL_AVAILABLE = True
except Exception:  # pyserial not installed
    serial = None
    SERIAL_AVAILABLE = False

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
#   Motor 3:  EN -> GPIO 10   STP -> GPIO 9    DIR -> GPIO 11
#   Motor 4:  EN -> GPIO 0    STP -> GPIO 5    DIR -> GPIO 6
#   (EN is active-LOW on the SERVO42C)
# ---------------------------------------------------------------------------
EN_PIN = 17
STP_PIN = 27
DIR_PIN = 22

EN2_PIN = 2
STP2_PIN = 3
DIR2_PIN = 4

EN3_PIN = 10
STP3_PIN = 9
DIR3_PIN = 11

EN4_PIN = 16
STP4_PIN = 20
DIR4_PIN = 21

# Gearbox reduction (motor revs : output revs). Motor 1 is direct-drive;
# motor 2 has a 5:1 planetary reducer, so its output shaft turns 5x slower
# than the motor shaft (and the encoder, which sits on the motor shaft).
# Motors 3 and 4 default to direct-drive (1:1); change if they have reducers.
MOTOR1_GEAR_RATIO = 1.0
MOTOR2_GEAR_RATIO = 5.0
MOTOR3_GEAR_RATIO = 1.0
MOTOR4_GEAR_RATIO = 1.0

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

    def __init__(self, en_pin, stp_pin, dir_pin, gear_ratio=1.0):
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
        else:
            self._en = self._step = self._dir = None

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

            # Position-hold: stay energized but emit no step pulses.
            if self._pause.is_set() and not self._soft_stop.is_set():
                self.current_speed = 0.0
                if self._step is not None:
                    self._step.value = 0.0
                time.sleep(RAMP_DT)
                continue

            # During a soft-stop the target eases to zero; once stopped, exit.
            target = 0.0 if self._soft_stop.is_set() else float(self.speed)
            step_accel = self.accel * dt
            if self.current_speed < target:
                self.current_speed = min(target, self.current_speed + step_accel)
            elif self.current_speed > target:
                self.current_speed = max(target, self.current_speed - step_accel)

            if self._soft_stop.is_set() and self.current_speed <= 0.0:
                break

            speed = max(self.current_speed, float(MIN_SPEED))
            if self._step is not None:
                self._step.frequency = max(1, int(round(speed)))
                self._step.value = 0.5  # 50% duty -> emit step pulses
            time.sleep(RAMP_DT)

        # Worker is exiting (soft ramp-down completed). Stop pulses, de-energize
        # and clear state so the motor is fully stopped without blocking.
        self.current_speed = 0.0
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
            "gpio": GPIO_AVAILABLE,
        }


motors = {
    1: StepperMotor(EN_PIN, STP_PIN, DIR_PIN, MOTOR1_GEAR_RATIO),
    2: StepperMotor(EN2_PIN, STP2_PIN, DIR2_PIN, MOTOR2_GEAR_RATIO),
    3: StepperMotor(EN3_PIN, STP3_PIN, DIR3_PIN, MOTOR3_GEAR_RATIO),
    4: StepperMotor(EN4_PIN, STP4_PIN, DIR4_PIN, MOTOR4_GEAR_RATIO),
}


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


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/emotion")
def emotion_page():
    return render_template("emotion.html")


@app.route("/emotion/latest")
def emotion_latest():
    """Latest face / emotion result.

    Placeholder until the camera + inference pipeline is added. Returns
    ``live: false`` so the UI falls back to its built-in simulation. When the
    camera is wired up, populate this with real values, e.g.::

        {
          "live": True,
          "face": 1,
          "box": [x, y, w, h],            # in the 640x480 frame
          "scores": {"happy": 0.0, "neutral": 0.0, "surprise": 0.0,
                     "sad": 0.0, "angry": 0.0, "fear": 0.0},
          "dominant": "neutral",
        }
    """
    return jsonify({
        "live": False,
        "face": None,
        "box": None,
        "scores": {
            "happy": 0.0, "neutral": 0.0, "surprise": 0.0,
            "sad": 0.0, "angry": 0.0, "fear": 0.0,
        },
        "dominant": None,
    })


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
