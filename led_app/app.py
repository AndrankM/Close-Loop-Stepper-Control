from flask import Flask, jsonify, render_template

app = Flask(__name__)

LED_TRIGGER = "/sys/class/leds/ACT/trigger"
LED_BRIGHTNESS = "/sys/class/leds/ACT/brightness"


def write_led(path, value):
    with open(path, "w") as f:
        f.write(value)


def read_brightness():
    with open(LED_BRIGHTNESS, "r") as f:
        return f.read().strip()


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


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
