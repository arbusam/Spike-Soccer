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
D_OFFSET            = -90# Compass correction (deg)
HIGH_STRENGTH        = 150    # Very strong IR signal
MED_STRENGTH        = 130    # Moderate IR signal
LOW_STRENGTH        = 120    # Weak IR signal
DIST_CLOSE            = 25    # cm threshold for back-left obstacle
DIST_FAR            = 90    # cm threshold for rear obstacle
MAX_SPEED            = 1110# Motor max speed
SLOW_SPEED            = 300# Backup / cautious speed
MEDIUM_SPEED        = 700 # Lost speed
YAW_CORRECT_SLOWDOWN = 50 # Slowdown for yaw correction (%)
YAW_CORRECT_SPEED    = 100# Speed for yaw correction
YAW_CORRECT_THRESHOLD = 50# Yaw correction threshold
LOOP_DELAY_MS        = 10    # Loop delay for cooperative multitasking

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

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # --- Yaw emergency correction ---
    yaw = hub.imu.heading()
    if yaw > YAW_CORRECT_THRESHOLD:# Rotated too far right, rotate left
        hub.light.on(Color.RED)
        a_value = a_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
        b_value = b_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
        c_value = c_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
        d_value = d_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
        if a_value > 1110:
            a_value = 1110
        elif a_value < -1110:
            a_value = -1110
        if b_value > 1110:
            b_value = 1110
        elif b_value < -1110:
            b_value = -1110
        if c_value > 1110:
            c_value = 1110
        elif c_value < -1110:
            c_value = -1110
        if d_value > 1110:
            d_value = 1110
        elif d_value < -1110:
            d_value = -1110

    elif yaw < -YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right
        hub.light.on(Color.ORANGE)
        a_value -= YAW_CORRECT_SPEED
        b_value -= YAW_CORRECT_SPEED
        c_value -= YAW_CORRECT_SPEED
        d_value -= YAW_CORRECT_SPEED
        if a_value > 1110:
            a_value = 1110
        elif a_value < -1110:
            a_value = -1110
        if b_value > 1110:
            b_value = 1110
        elif b_value < -1110:
            b_value = -1110
        if c_value > 1110:
            c_value = 1110
        elif c_value < -1110:
            c_value = -1110
        if d_value > 1110:
            d_value = 1110
        elif d_value < -1110:
            d_value = -1110
    else:
        hub.light.off()

    # print(a_mult, b_mult, c_mult, d_mult, speed)
    a_motor.run(a_value)
    b_motor.run(b_value)
    c_motor.run(c_value)
    d_motor.run(d_value)

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
# --- Read sensors ---
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
        if dir == 0:
            hub.display.char("C")
            move(finalDirection, MEDIUM_SPEED)
            hub.light.on(Color.VIOLET)
            continue
        # --- skip when no IR signal ---

        distance = us.distance() / 10

        if dir == 0:
            hub.display.char("C")
        elif dir == 1:
            hub.display.number(1)
        elif dir == 2:
            hub.display.number(2)
        elif dir == 3:
            hub.display.number(3)
        elif dir == 4:
            hub.display.number(4)
        elif dir == 5:
            hub.display.number(5)
        elif dir == 6:
            hub.display.number(6)
        elif dir == 7:
            hub.display.number(7)
        elif dir == 8:
            hub.display.number(8)
        elif dir == 9:
            hub.display.number(9)
        elif dir == 10:
            hub.display.number(10)
        elif dir == 11:
            hub.display.number(11)
        elif dir == 12:
            hub.display.number(12)
        elif dir == 13:
            hub.display.number(13)
        elif dir == 14:
            hub.display.number(14)
        elif dir == 15:
            hub.display.number(15)
        elif dir == 16:
            hub.display.number(16)
        elif dir == 17:
            hub.display.number(17)
        elif dir == 18:
            hub.display.number(18)

        speed = MAX_SPEED
        if str > HIGH_STRENGTH:
            speed = SLOW_SPEED
        elif str == 0: #Go backwards
            speed = SLOW_SPEED
            finalDirection = 280
        #Forward Directional Commands
        if dir in (10, 11, 12, 13, 14, 15, 16, 17, 18) and str >= HIGH_STRENGTH:
            finalDirection = 90
        if dir == 14:# Forward
            finalDirection = 90
        elif dir == 15 and str < HIGH_STRENGTH:
            finalDirection = 130
        elif dir == 13:
            finalDirection = 80
        elif dir == 8:
            finalDirection = 300
        elif dir == 10:
            finalDirection = 330
        elif dir == 11:
            finalDirection = 360
        elif dir == 12 and str < HIGH_STRENGTH:
            finalDirection = 40
        elif dir == 16 and str < HIGH_STRENGTH:
            finalDirection = 140
        elif dir == 17:# Front Right
            finalDirection = 180
        #Backwards Directional Commands
        elif dir in (5, 6, 7):# Backward
            finalDirection = 320
        elif dir == 18:
            finalDirection = 200
        elif dir == 4:# BackBackRight
            finalDirection = 320
        elif dir == 1:# BackRight
            finalDirection = 300
        elif dir == 3:
            finalDirection = 280
        #East-West Directional Commands
        elif dir == 2:# Right
            finalDirection = 280
        elif dir == 9:# Left
            finalDirection = 310
        finalDirection += D_OFFSET
        move(finalDirection, speed)
        print([dir, speed, str, finalDirection])
        wait(LOOP_DELAY_MS) # Delay

main()