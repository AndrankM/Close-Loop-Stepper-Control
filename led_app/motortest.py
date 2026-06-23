"""Standalone hardware step test for NEMA17 + MKS SERVO42C.

Bypasses Flask. Enables the driver, sets direction, and emits a burst of
clean step pulses with microsecond timing using lgpio directly.

  EN  -> GPIO17
  STP -> GPIO27
  DIR -> GPIO22

Usage:
  python motortest.py [steps] [pulse_us] [dir] [en_active_low]
    steps         : number of step pulses (default 3200 = 1 rev at 1/16)
    pulse_us      : half-period microseconds (default 500 -> 1kHz)
    dir           : 0 or 1 (default 1)
    en_active_low : 1 (EN low = enabled, default) or 0 (EN high = enabled)
"""
import sys
import time

import lgpio

EN_PIN = 17
STP_PIN = 27
DIR_PIN = 22

steps = int(sys.argv[1]) if len(sys.argv) > 1 else 3200
pulse_us = int(sys.argv[2]) if len(sys.argv) > 2 else 500
direction = int(sys.argv[3]) if len(sys.argv) > 3 else 1
en_active_low = int(sys.argv[4]) if len(sys.argv) > 4 else 1

en_enabled_level = 0 if en_active_low else 1
en_disabled_level = 1 if en_active_low else 0

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, EN_PIN, en_disabled_level)
lgpio.gpio_claim_output(h, STP_PIN, 0)
lgpio.gpio_claim_output(h, DIR_PIN, direction)

print(f"EN=GPIO{EN_PIN} STP=GPIO{STP_PIN} DIR=GPIO{DIR_PIN}")
print(f"steps={steps} pulse_us={pulse_us} dir={direction} "
      f"en_active_low={en_active_low}")

try:
    # Enable driver and give it a moment to energize the coils.
    lgpio.gpio_write(h, EN_PIN, en_enabled_level)
    print("Driver ENABLED. Holding 1s (coils should now hold torque)...")
    time.sleep(1.0)

    print("Pulsing STEP...")
    half = pulse_us / 1_000_000.0
    for _ in range(steps):
        lgpio.gpio_write(h, STP_PIN, 1)
        time.sleep(half)
        lgpio.gpio_write(h, STP_PIN, 0)
        time.sleep(half)

    print("Done pulsing. Holding enabled 1s...")
    time.sleep(1.0)
finally:
    lgpio.gpio_write(h, EN_PIN, en_disabled_level)
    lgpio.gpiochip_close(h)
    print("Driver DISABLED. Exit.")
