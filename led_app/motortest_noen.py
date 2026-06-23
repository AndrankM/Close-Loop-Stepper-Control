"""Step test that does NOT drive EN at all (leaves it floating).

Mirrors controllers that only wire STEP + DIR and leave ENABLE
disconnected. Useful for the MKS SERVO42C where EN is often left open.

  STP -> GPIO27
  DIR -> GPIO22
  (EN  -> GPIO17 is intentionally NOT claimed/driven)

Usage:
  python motortest_noen.py [steps] [pulse_us] [dir]
"""
import sys
import time

import lgpio

STP_PIN = 27
DIR_PIN = 22

steps = int(sys.argv[1]) if len(sys.argv) > 1 else 3200
pulse_us = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
direction = int(sys.argv[3]) if len(sys.argv) > 3 else 1

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, STP_PIN, 0)
lgpio.gpio_claim_output(h, DIR_PIN, direction)

print(f"STP=GPIO{STP_PIN} DIR=GPIO{DIR_PIN}  (EN left floating)")
print(f"steps={steps} pulse_us={pulse_us} dir={direction}")

try:
    print("Pulsing STEP...")
    half = pulse_us / 1_000_000.0
    for _ in range(steps):
        lgpio.gpio_write(h, STP_PIN, 1)
        time.sleep(half)
        lgpio.gpio_write(h, STP_PIN, 0)
        time.sleep(half)
    print("Done.")
finally:
    lgpio.gpiochip_close(h)
    print("Exit.")
