import time
import sys

LED_TRIGGER = "/sys/class/leds/ACT/trigger"
LED_BRIGHTNESS = "/sys/class/leds/ACT/brightness"
INTERVAL = 0.5


def write_led(path, value):
    with open(path, "w") as f:
        f.write(value)


def blink():
    print("Disabling default LED trigger...")
    write_led(LED_TRIGGER, "none")

    print("Blinking onboard ACT LED indefinitely. Press Ctrl+C to stop.")
    i = 0
    try:
        while True:
            i += 1
            write_led(LED_BRIGHTNESS, "1")
            print(f"  [{i}] ON")
            time.sleep(INTERVAL)
            write_led(LED_BRIGHTNESS, "0")
            print(f"  [{i}] OFF")
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Restoring default LED trigger (mmc0)...")
        write_led(LED_TRIGGER, "mmc0")
        print("Done.")


if __name__ == "__main__":
    blink()
