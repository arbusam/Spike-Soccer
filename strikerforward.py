from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

MAX_SPEED = 1050
MAX_ACCELERATION = 2000

RUN_DURATION_MS = 5000
SAMPLE_INTERVAL_MS = 10
SAMPLED_MOTOR_PORTS = (Port.E, Port.A, Port.C, Port.D)

QUADRANT_FUNCS = [
    # E, A, C, D
    lambda r: (1-r, -1, 1, r-1),    # 0°-89° N → E
    lambda r: (-1, r-1, 1-r, 1),    # 90°-179° E → S
    lambda r: (r-1, 1, -1, 1-r),    # 180°-269° S → W
    lambda r: (1, 1-r, r-1, -1),    # 270°-359° W → N
]

# --------------------------------------------
# Device initialization
# --------------------------------------------

e_motor = Motor(Port.E)
a_motor = Motor(Port.A)
c_motor = Motor(Port.C)
d_motor = Motor(Port.D)
ir_sensor = PUPDevice(Port.B)
us = UltrasonicSensor(Port.F)
hub = PrimeHub()

PORT_TO_MOTOR = {
    Port.E: e_motor,
    Port.A: a_motor,
    Port.C: c_motor,
    Port.D: d_motor,
}

PORT_LABELS = {
    Port.E: "E",
    Port.A: "A",
    Port.C: "C",
    Port.D: "D",
}


print(a_motor.control.limits())
e_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
a_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
c_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
d_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)

# e_motor.control.pid(21242, 21242, 10620, 8, 15)
# a_motor.control.pid(21242, 21242, 10620, 8, 15)
# c_motor.control.pid(21242, 21242, 10620, 8, 15)
# d_motor.control.pid(21242, 21242, 10620, 8, 15)

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for quadrant vectors ---
    quadrant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    e_mult, a_mult, c_mult, d_mult = QUADRANT_FUNCS[quadrant](ratio)
    e_value = int(e_mult * speed)
    a_value = int(a_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    e_motor.run(e_value)
    a_motor.run(a_value)
    c_motor.run(c_value)
    d_motor.run(d_value)

speed_samples = {port: [] for port in SAMPLED_MOTOR_PORTS}
heading_samples = []
timer = StopWatch()
timer.reset()

hub.imu.reset_heading(0)

while timer.time() < RUN_DURATION_MS:
    move(0, MAX_SPEED)
    heading = hub.imu.heading("3D")
    heading = ((heading + 180) % 360) - 180
    heading_samples.append(heading)
    for port in SAMPLED_MOTOR_PORTS:
        speed_samples[port].append(PORT_TO_MOTOR[port].speed())
    wait(SAMPLE_INTERVAL_MS)

e_motor.stop()
a_motor.stop()
c_motor.stop()
d_motor.stop()

run_data = {
    "sample_interval_ms": SAMPLE_INTERVAL_MS,
    "reference_speed": MAX_SPEED,
    "heading_samples": heading_samples,
    "speed_samples": {
        PORT_LABELS[port]: samples for port, samples in speed_samples.items()
    },
}

print("Run data:", run_data)
