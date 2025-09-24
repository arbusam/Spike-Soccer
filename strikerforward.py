from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

MAX_SPEED = 100

QUADRANT_FUNCS = [
    #E, F, C, D
    lambda r: (1-r, -1, 1, r-1),    # 0°-89° N → E
    lambda r: (-1, r-1, 1-r, 1),    # 90°-179° E → S
    lambda r: (r-1, 1, -1, 1-r),    # 180°-269° S → W
    lambda r: (1, 1-r, r-1, -1),    # 270°-359° W → N
]

# --------------------------------------------
# Device initialization
# --------------------------------------------

e_motor = Motor(Port.E, Direction.CLOCKWISE, [36, 12])
f_motor = Motor(Port.F, Direction.CLOCKWISE, [36, 12])
c_motor = Motor(Port.C, Direction.CLOCKWISE, [36, 12])
d_motor = Motor(Port.D, Direction.CLOCKWISE, [36, 12])
ir_sensor = PUPDevice(Port.B)
us = UltrasonicSensor(Port.A)


print(f_motor.control.limits())

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for quadrant vectors ---
    quadrant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    e_mult, f_mult, c_mult, d_mult = QUADRANT_FUNCS[quadrant](ratio)
    e_value = int(e_mult * speed)
    f_value = int(f_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    e_motor.dc(e_value)
    f_motor.dc(f_value)
    c_motor.dc(c_value)
    d_motor.dc(d_value)

while True:
    move(0, MAX_SPEED)
    print(e_motor.speed(), f_motor.speed(), c_motor.speed(), d_motor.speed())
    wait(10)