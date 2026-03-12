import serial
from serial import SerialException
from serial.tools import list_ports
import matplotlib.pyplot as plt
import time

PORT = "COM5"
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except SerialException as exc:
    print(f"Could not open {PORT}: {exc}")
    ports = [p.device for p in list_ports.comports()]
    if ports:
        print("Available serial ports:", ", ".join(ports))
    else:
        print("No serial ports detected.")
    raise SystemExit(1)

time.sleep(2)

sp = []
w = []
t = []

start = time.time()

for i in range(2000):

    line = ser.readline().decode(errors="ignore").strip()
    values = line.split(",")

    if len(values) < 2:
        continue

    try:
        sp_val = float(values[0])
        w_val = float(values[1])

        sp.append(sp_val)
        w.append(w_val)
        t.append(time.time() - start)

    except ValueError:
        continue

ser.close()

plt.plot(t, sp, label="Setpoint")
plt.plot(t, w, label="Omega")

plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.legend()
plt.grid()

plt.show()