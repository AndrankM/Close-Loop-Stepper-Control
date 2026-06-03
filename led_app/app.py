import os
import threading
import time

from flask import Flask, jsonify, render_template, request

try:
    from gpiozero import DigitalOutputDevice

    GPIO_AVAILABLE = True
except Exception:  # gpiozero not installed / not running on a Pi
    DigitalOutputDevice = None
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
# NEMA 17 + MKS SERVO42C stepper control
#   EN  -> GPIO 17   (enable, active-LOW on the SERVO42C)
#   STP -> GPIO 27   (step pulse)
#   DIR -> GPIO 22   (direction)
# ---------------------------------------------------------------------------
EN_PIN = 17
STP_PIN = 27
DIR_PIN = 22

# Driver enable pin is active-LOW: drive LOW to energize the coils.
EN_ACTIVE_LOW = True

# Speed limits in steps per second.
MIN_SPEED = 1
MAX_SPEED = 2000
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


class StepperMotor:
    """Generates step pulses on a background thread while the motor is enabled."""

    def __init__(self):
        self._lock = threading.Lock()
        self.enabled = False
        self.stopping = False
        self.direction = "cw"  # "cw" or "ccw"
        self.speed = DEFAULT_SPEED  # target steps per second
        self.current_speed = 0.0  # live (ramped) steps per second
        self.accel = DEFAULT_ACCEL  # steps per second^2
        self.full_steps_per_rev = DEFAULT_FULL_STEPS_PER_REV
        self.microstepping = DEFAULT_MICROSTEPPING
        self._stop = threading.Event()
        self._soft_stop = threading.Event()
        self._thread = None

        if GPIO_AVAILABLE:
            # The device handles active-low inversion so "off" leaves the
            # driver disabled (EN pin held HIGH).
            self._en = DigitalOutputDevice(
                EN_PIN, active_high=not EN_ACTIVE_LOW, initial_value=False
            )
            self._step = DigitalOutputDevice(STP_PIN, initial_value=False)
            self._dir = DigitalOutputDevice(DIR_PIN, initial_value=False)
        else:
            self._en = self._step = self._dir = None

    # -- worker -------------------------------------------------------------
    def _run(self):
        # Ramp current_speed toward target self.speed by self.accel each step,
        # then time the pulse for the ramped speed.
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = now - last
            last = now

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
            half = 1.0 / (2.0 * speed)
            if self._step is not None:
                self._step.on()
                time.sleep(half)
                self._step.off()
                time.sleep(half)
            else:
                time.sleep(half * 2)

        # Worker is exiting (soft ramp-down completed). De-energize and clear
        # state so the motor is fully stopped without blocking the caller.
        self.current_speed = 0.0
        if self._en is not None:
            self._en.off()
        with self._lock:
            self.enabled = False
            self.stopping = False

    # -- public API ---------------------------------------------------------
    def enable(self):
        with self._lock:
            if self.enabled:
                return
            self.enabled = True
            self.stopping = False
            self.current_speed = 0.0
            self._apply_direction()
            if self._en is not None:
                self._en.on()  # energize (handles active-low)
            self._stop.clear()
            self._soft_stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

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
        rpm = float(rpm)
        with self._lock:
            speed = round(rpm * self._steps_per_rev() / 60.0)
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
        return round(self.speed * 60.0 / self._steps_per_rev(), 2)

    def _current_rpm(self):
        return round(self.current_speed * 60.0 / self._steps_per_rev(), 2)

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
            "gpio": GPIO_AVAILABLE,
        }


motor = StepperMotor()


# ---------------------------------------------------------------------------
# SERVO42C UART encoder reader
#   Wiring (Pi <-> SERVO42C TTL header):
#     Pi TXD (GPIO14) -> SERVO42C Rx
#     Pi RXD (GPIO15) -> SERVO42C Tx
#     Pi GND          -> SERVO42C G
#   Read encoder command (manual 5.1.1): send "ADDR 30 CRC", returns 8 bytes:
#     ADDR + carry(int32, big-endian) + value(uint16, big-endian) + CRC
#   CRC is checksum-8 (sum of preceding bytes & 0xFF).
#   The encoder updates in any work mode (incl. the default CR_vFOC), so this
#   works alongside STP/DIR motion without switching to CR_UART.
# ---------------------------------------------------------------------------
SERIAL_PORT = os.environ.get("SERVO_UART", "/dev/serial0")
SERIAL_BAUD = int(os.environ.get("SERVO_BAUD", "9600"))
MOTOR_ADDR = int(os.environ.get("SERVO_ADDR", "0xe0"), 0)
ENCODER_COUNTS_PER_REV = 65536  # 0~0xFFFF maps to 0~360 degrees
READ_ENCODER_CMD = 0x30


class EncoderReader:
    """Reads the SERVO42C magnetic encoder over the UART (TTL) port."""

    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD, addr=MOTOR_ADDR):
        self.port = port
        self.baud = baud
        self.addr = addr & 0xFF
        self._lock = threading.Lock()
        self._ser = None
        self.init_error = None
        if not SERIAL_AVAILABLE:
            self.init_error = "pyserial not installed"
        else:
            try:
                self._ser = serial.Serial(port, baud, timeout=0.2)
            except Exception as exc:  # port missing / permission denied
                self.init_error = str(exc)

    @property
    def available(self):
        return self._ser is not None

    @staticmethod
    def _checksum(data):
        return sum(data) & 0xFF

    def read(self):
        if self._ser is None:
            return {"available": False, "error": self.init_error}
        cmd = bytes([self.addr, READ_ENCODER_CMD])
        packet = cmd + bytes([self._checksum(cmd)])
        with self._lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(packet)
                # Accumulate until 8 bytes arrive or the deadline passes, so a
                # short per-read timeout doesn't truncate a slow reply.
                resp = bytearray()
                deadline = time.time() + 0.5
                while len(resp) < 8 and time.time() < deadline:
                    chunk = self._ser.read(8 - len(resp))
                    if chunk:
                        resp.extend(chunk)
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
        return {
            "available": True,
            "error": None,
            "carry": carry,
            "value": value,
            "counts": counts,
            "angle_deg": round(angle, 2),
            "total_angle_deg": round(total_angle, 2),
            "revolutions": round(revolutions, 4),
        }


encoder = EncoderReader()


@app.route("/")
def index():
    return render_template("index.html")


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


# -- Stepper motor ----------------------------------------------------------
@app.route("/motor/enable", methods=["POST"])
def motor_enable():
    motor.enable()
    return jsonify(motor.status())


@app.route("/motor/disable", methods=["POST"])
def motor_disable():
    data = request.get_json(silent=True) or {}
    # Default to a soft stop (ramp down); pass {"soft": false} for an
    # immediate hard stop / emergency cut.
    soft = data.get("soft", True)
    motor.disable(soft=bool(soft))
    return jsonify(motor.status())


@app.route("/motor/direction", methods=["POST"])
def motor_direction():
    data = request.get_json(silent=True) or {}
    direction = data.get("direction", "cw")
    try:
        motor.set_direction(direction)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify(motor.status())


@app.route("/motor/speed", methods=["POST"])
def motor_speed():
    data = request.get_json(silent=True) or {}
    try:
        motor.set_speed(data.get("speed", DEFAULT_SPEED))
    except (TypeError, ValueError):
        return jsonify({"error": "speed must be an integer"}), 400
    return jsonify(motor.status())


@app.route("/motor/rpm", methods=["POST"])
def motor_rpm():
    data = request.get_json(silent=True) or {}
    try:
        motor.set_rpm(data.get("rpm", 0))
    except (TypeError, ValueError):
        return jsonify({"error": "rpm must be a number"}), 400
    return jsonify(motor.status())


@app.route("/motor/geometry", methods=["POST"])
def motor_geometry():
    data = request.get_json(silent=True) or {}
    try:
        motor.set_geometry(
            full_steps_per_rev=data.get("full_steps_per_rev"),
            microstepping=data.get("microstepping"),
        )
    except (TypeError, ValueError) as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify(motor.status())


@app.route("/motor/accel", methods=["POST"])
def motor_accel():
    data = request.get_json(silent=True) or {}
    try:
        motor.set_accel(data.get("accel", DEFAULT_ACCEL))
    except (TypeError, ValueError):
        return jsonify({"error": "accel must be an integer"}), 400
    return jsonify(motor.status())


@app.route("/motor/status")
def motor_status():
    return jsonify(motor.status())


@app.route("/motor/encoder")
def motor_encoder():
    return jsonify(encoder.read())


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000)
    finally:
        motor.disable(soft=False)
