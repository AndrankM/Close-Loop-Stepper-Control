import spidev
import time


def enc(colors, brightness=80):
    scale = brightness / 255.0
    bits = []
    for (r, g, b) in colors:
        for byte in (int(g * scale), int(r * scale), int(b * scale)):
            for i in range(7, -1, -1):
                bits.extend((1, 1, 0) if (byte >> i) & 1 else (1, 0, 0))
    out = bytearray()
    for i in range(0, len(bits), 8):
        chunk = bits[i:i + 8]
        while len(chunk) < 8:
            chunk.append(0)
        v = 0
        for bit in chunk:
            v = (v << 1) | bit
        out.append(v)
    out.extend(b"\x00" * 40)
    return out


spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2400000
spi.mode = 0
N = 8

print("RED")
spi.writebytes2(enc([(255, 0, 0)] * N))
time.sleep(1.0)
print("GREEN")
spi.writebytes2(enc([(0, 255, 0)] * N))
time.sleep(1.0)
print("BLUE")
spi.writebytes2(enc([(0, 0, 255)] * N))
time.sleep(1.0)
print("OFF")
spi.writebytes2(enc([(0, 0, 0)] * N))
spi.close()
print("done")
