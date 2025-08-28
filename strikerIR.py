from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET                     = -90  # Compass correction (deg)
HIGH_STRENGTH                = 150  # Very strong IR signal
MED_STRENGTH                 = 130  # Moderate IR signal
LOW_STRENGTH                 = 120  # Weak IR signal
DIST_CLOSE                   = 25   # cm threshold for back-left obstacle
DIST_FAR                     = 90   # cm threshold for rear obstacle
MAX_SPEED                    = 1110 # Motor max speed
SLOW_SPEED                   = 300  # Backup / cautious speed
MEDIUM_SPEED                 = 700  # Lost speed
YAW_CORRECT_SLOWDOWN         = 50   # Slowdown for fast dynamic yaw correction (%)
YAW_CORRECT_SPEED            = 200  # Speed for fast dynamic yaw correction
YAW_CORRECT_THRESHOLD        = 15   # Fast dynamic yaw correction threshold
STATIC_YAW_CORRECT_THRESHOLD = 50   # Yaw correct threshold for static
STATIC_YAW_CORRECT_SPEED     = 500  # Static yaw correct speed
SLOW_YAW_CORRECT_SLOWDOWN    = 75   # Slowdown for slow dynamic yaw correction (%)
SLOW_YAW_CORRECT_SPEED       = 50   # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_THRESHOLD   = 8    # Slow dynamic yaw correction threshold
LOOP_DELAY_MS                = 10   # Loop delay for cooperative multitasking

# Inputs: octant (0-7) and ratio (0-1)
# Octant: the sector of the full 360 degree circle in which the direction lies.
# Ratio: the position within that octant, where 0 is the start and 1 is the end.
# Outputs: a multiplier for each of the four motors (-1 to 1).

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

a_motor = Motor(Port.E)
b_motor = Motor(Port.F)
c_motor = Motor(Port.C)
d_motor = Motor(Port.D)
hub = PrimeHub()
ir_sensor = PUPDevice(Port.B)
us = UltrasonicSensor(Port.A)

a_motor.control.limits(MAX_SPEED)
b_motor.control.limits(MAX_SPEED)
c_motor.control.limits(MAX_SPEED)
d_motor.control.limits(MAX_SPEED)

def Ir_Combine_360_Sensor_Data(FrontDirection, FrontStrength, BackDirection, BackStrength):
    Direction, SignalStrength = 0, 0
    if (FrontStrength == 0 and BackStrength == 0):
        Direction = 0
    else:
        if (FrontStrength > BackStrength):
            Direction = round(FrontDirection) + 9
            SignalStrength = round(FrontStrength)
        else:
            Direction = round(BackDirection)
            SignalStrength = round(BackStrength)
    return Direction, SignalStrength

def Ir_Read_360_Sensor_Data(ReductionFactor):
    BackDirection = ir_sensor.read(1)[0]
    BackStrength, FrontStrength, FrontDirection = ir_sensor.read(5)[:3]
    return Ir_Combine_360_Sensor_Data(FrontDirection//ReductionFactor, FrontStrength//ReductionFactor, BackDirection//ReductionFactor, BackStrength//ReductionFactor)

def main():
    stop = False
    pressed = False
    timer = 0
    finalDirection = 90
    hub.imu.reset_heading(0)
    while True:
        # --- Stop Button ---
        timer += LOOP_DELAY_MS
        if pressed:
            if Button.RIGHT not in hub.buttons.pressed():
                pressed = False
            else:
                continue
        elif Button.RIGHT in hub.buttons.pressed():
            stop = not stop
            pressed = True

        if stop:
            hub.display.char("S")
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.brake()
            continue

        dir, str = Ir_Read_360_Sensor_Data(4)
        if 1 <= dir <= 18:
            hub.display.number(dir)
        else:
            hub.display.char("C")
            hub.light.on(Color.VIOLET)
            continue

        distance = us.distance() / 10

        print([dir, "speed", str, finalDirection])
        wait(LOOP_DELAY_MS) # Delay
main()
