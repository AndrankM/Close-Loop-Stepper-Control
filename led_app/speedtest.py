import json
import time
import urllib.request

BASE = "http://localhost:5000"


def post(path, body=None):
    data = json.dumps(body).encode() if body is not None else b""
    req = urllib.request.Request(
        BASE + path, data=data,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    return json.load(urllib.request.urlopen(req))


def counts(mid):
    return json.load(urllib.request.urlopen(f"{BASE}/motor/{mid}/encoder"))["counts"]


def measure(mid, speed, secs=3.0):
    post(f"/motor/{mid}/speed", {"speed": speed})
    post(f"/motor/{mid}/enable")
    time.sleep(1.0)
    a = counts(mid)
    t0 = time.time()
    time.sleep(secs)
    b = counts(mid)
    dt = time.time() - t0
    d = b - a
    motor_rpm = d / 65536 / dt * 60
    print(f"motor {mid} speed={speed:>4}: motor_rpm={motor_rpm:7.2f}  "
          f"counts/s={d/dt:9.0f}")


for mid in (1, 2):
    for sp in (1000, 2000, 4000, 6000):
        measure(mid, sp)
    post(f"/motor/{mid}/disable", {"soft": False})
    print()
