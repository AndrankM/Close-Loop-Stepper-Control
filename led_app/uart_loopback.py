import time

import serial

s = serial.Serial("/dev/serial0", 9600, timeout=0.5)
s.reset_input_buffer()
s.write(b"hello")
time.sleep(0.1)
print("GOT:", repr(s.read(5)))
s.close()
