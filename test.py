from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port

QUADRANT_FUNCS = [
    lambda r: (r-1, 1, -1, 1-r),    # 0°‑89° N → E
    lambda r: (1, 1-r, r-1, -1),    # 90°‑179° E → S
    lambda r: (1-r, -1, 1, r-1),    # 180°‑270° S → W
    lambda r: (-1, r-1, 1-r, 1),    # 270°‑359° W → N
]

# ---------------------------------------------
# Motor helper
# ---------------------------------------------
a_motor = Motor(Port.A)
b_motor = Motor(Port.B)
c_motor = Motor(Port.C)
d_motor = Motor(Port.D)

a_motor.control.limits(1110, 5000)
b_motor.control.limits(1110, 5000)
c_motor.control.limits(1110, 5000)
d_motor.control.limits(1110, 5000)

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)

    a_motor.run(int(a_mult * speed))
    b_motor.run(int(b_mult * speed))
    c_motor.run(int(c_mult * speed))
    d_motor.run(int(d_mult * speed))

def main():
  while True:
    move(0, 1110)

main()