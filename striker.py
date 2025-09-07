from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET                     = 0  # Compass correction (deg)
TOUCHING_STRENGTH            = 185  # IR Strength for touching ball
HIGH_STRENGTH                = 165  # Very strong IR signal
MED_STRENGTH                 = 130  # Moderate IR signal
LOW_STRENGTH                 = 120  # Weak IR signal
DIST_CLOSE                   = 25   # cm threshold for back-left obstacle
DIST_FAR                     = 90   # cm threshold for rear obstacle
MAX_SPEED                    = 1110 # Motor max speed
SLOW_SPEED                   = 300  # Backup / cautious speed
MEDIUM_SPEED                 = 700  # Lost speed
TOUCHING_SPEED               = 400  # Speed when touching ball
YAW_CORRECT_SLOWDOWN         = 50   # Slowdown for fast dynamic yaw correction (%)
YAW_CORRECT_SPEED            = 200  # Speed for fast dynamic yaw correction
YAW_CORRECT_THRESHOLD        = 15   # Fast dynamic yaw correction threshold
STATIC_YAW_CORRECT_THRESHOLD = 50   # Yaw correct threshold for static
STATIC_YAW_CORRECT_SPEED     = 1110 # Static yaw correct speed
SLOW_YAW_CORRECT_SLOWDOWN    = 75   # Slowdown for slow dynamic yaw correction (%)
SLOW_YAW_CORRECT_SPEED       = 50   # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_THRESHOLD   = 8    # Slow dynamic yaw correction threshold
LOOP_DELAY_MS                = 10   # Loop delay for cooperative multitasking
RIGHT_STEERING_THRESHOLD     = 100  # Threshold for right steering
LEFT_STEERING_THRESHOLD      = 80   # Threshold for left steering
STEERING_ANGULAR_DIRECTION   = 30   # The direction of steering in either direction
HOLDING_BALL_THRESHOLD       = 190  # Threshold after which the bot is considered to be 'holding' the ball
STRENGTH_CONVERSION_FACTOR   = 2.5  # Factor to convert striker strength to defence for communication

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
hub = PrimeHub(observe_channels=[77], broadcast_channel=37)
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

    # --- Lookup table for quadrant vectors ---
    quadrant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[quadrant](ratio)
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # --- Dynamic yaw correction ---
    yaw = hub.imu.heading("3D")
    yaw = ((yaw + 180) % 360) - 180  # Normalize to [-180, 180)
    if yaw > SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far right, rotate left
        hub.light.on(Color.ORANGE)
        if yaw > YAW_CORRECT_THRESHOLD:
            a_value = a_value * YAW_CORRECT_SLOWDOWN // 100 - YAW_CORRECT_SPEED
            b_value = b_value * YAW_CORRECT_SLOWDOWN // 100 - YAW_CORRECT_SPEED
            c_value = c_value * YAW_CORRECT_SLOWDOWN // 100 - YAW_CORRECT_SPEED
            d_value = d_value * YAW_CORRECT_SLOWDOWN // 100 - YAW_CORRECT_SPEED

        else:
            a_value = a_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
            b_value = b_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
            c_value = c_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
            d_value = d_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
            

    elif yaw < -SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right
        hub.light.on(Color.ORANGE)
        if yaw < -YAW_CORRECT_THRESHOLD:
            a_value = a_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
            b_value = b_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
            c_value = c_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
            d_value = d_value * YAW_CORRECT_SLOWDOWN // 100 + YAW_CORRECT_SPEED
        else:
            a_value = a_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
            b_value = b_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
            c_value = c_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
            d_value = d_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED

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

# No encoding/decoding needed: BLE can send/receive str or int directly.

def main():
    stop = True
    pressed = False
    finalDirection = 0
    message = None
    hub.imu.reset_heading(0)
    stopwatch = StopWatch()
    while True:
        initial_time = stopwatch.time()
        # --- Stop Button ---
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
            hub.ble.broadcast(None)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.brake()
            continue

        # --- Static yaw correction ---
        yaw = hub.imu.heading("3D")
        yaw = ((yaw + 180) % 360) - 180
        if abs(yaw) > STATIC_YAW_CORRECT_THRESHOLD:
            while abs(yaw) > YAW_CORRECT_THRESHOLD:
                if yaw > STATIC_YAW_CORRECT_THRESHOLD:
                    hub.light.on(Color.RED)
                    for motor in (a_motor, b_motor, c_motor, d_motor):
                        motor.run(-STATIC_YAW_CORRECT_SPEED)
                elif yaw < -STATIC_YAW_CORRECT_THRESHOLD:
                    hub.light.on(Color.RED)
                    for motor in (a_motor, b_motor, c_motor, d_motor):
                        motor.run(STATIC_YAW_CORRECT_SPEED)
                yaw = hub.imu.heading("3D")
                yaw = ((yaw + 180) % 360) - 180
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.hold()

        message = hub.ble.observe(77)
        defence_strength = -1
        if isinstance(message, int):
            defence_strength = message
            message = None
        message_to_broadcast = None

        # --- Read sensors ---
        dir, strength = Ir_Read_360_Sensor_Data(4)
        distance = us.distance() / 10

        if 1 <= dir <= 18:
            hub.display.number(dir)
        else:
            if dir == 0:
                hub.ble.broadcast("C")
                hub.display.char("C")
                if message == "T" or (defence_strength != -1):
                    if distance > RIGHT_STEERING_THRESHOLD:
                        speed = SLOW_SPEED
                        finalDirection = 90
                    elif distance < LEFT_STEERING_THRESHOLD:
                        speed = SLOW_SPEED
                        finalDirection = -90
                    move(finalDirection, MEDIUM_SPEED)
                    hub.light.on(Color.BLACK)
                    continue
                else:
                    move(finalDirection, MEDIUM_SPEED)
                    hub.light.on(Color.VIOLET)
                    continue

        speed = MAX_SPEED
        if strength > HIGH_STRENGTH:
            speed = SLOW_SPEED
        #Forward Directional Commands
        if dir in (13, 14, 15) and strength >= TOUCHING_STRENGTH:
            if strength >= HOLDING_BALL_THRESHOLD:
                message_to_broadcast = "T"
                if distance > RIGHT_STEERING_THRESHOLD:
                    speed = MAX_SPEED
                    finalDirection = STEERING_ANGULAR_DIRECTION
                    hub.display.char("R")
                elif distance < LEFT_STEERING_THRESHOLD:
                    speed = MAX_SPEED
                    finalDirection = 360 - STEERING_ANGULAR_DIRECTION
                    hub.display.char("L")
            else:
                finalDirection = 0
        elif dir in (1, 2, 3, 4, 5, 6, 7, 8) and strength >= HIGH_STRENGTH:
            finalDirection = 55
            message_to_broadcast = "O"
        elif dir == 14:# Forward
            finalDirection = 0
        elif dir == 15 and strength < HIGH_STRENGTH:
            finalDirection = 40
            speed = MEDIUM_SPEED
        elif dir == 13:
            finalDirection = 345
            speed = MEDIUM_SPEED
        elif dir == 8 and strength >= MED_STRENGTH:
            finalDirection = 160
            message_to_broadcast = "O"
        elif dir == 10 and strength >= MED_STRENGTH:
            finalDirection = 220
            message_to_broadcast = "O"
        elif dir == 11:
            finalDirection = 280
        elif dir == 12: #Double check
            finalDirection = 325
        elif dir == 16 and strength < HIGH_STRENGTH:
            finalDirection = 45
        elif dir == 17:# Front Right
            finalDirection = 90
        #Backwards Directional Commands
        elif dir == 3 and strength < HIGH_STRENGTH:
            finalDirection = 220
            message_to_broadcast = "O"
        elif dir == 4 and strength < HIGH_STRENGTH:
            finalDirection = 250
            message_to_broadcast = "O"
        elif dir == 5 and strength < HIGH_STRENGTH:
            finalDirection = 130
            message_to_broadcast = "O"
        elif dir == 6 and strength < MED_STRENGTH:
            finalDirection = 140
            message_to_broadcast = "O"
        elif dir == 7 and strength < MED_STRENGTH:
            finalDirection = 150
            message_to_broadcast = "O"
        elif dir == 18:
            finalDirection = 120
        elif dir == 1 and strength < MED_STRENGTH:# BackRight
            message_to_broadcast = "O"
            finalDirection = 150
        #East-West Directional Commands
        elif dir == 2 and strength < MED_STRENGTH:# Right
            finalDirection = 170
            message_to_broadcast = "O"
        elif dir == 9 and strength < MED_STRENGTH:# Left
            finalDirection = 160
            message_to_broadcast = "O"
        finalDirection += D_OFFSET
        move(finalDirection, speed)
        print([dir, speed, strength, finalDirection])
        if message_to_broadcast is None:
            message_to_broadcast = int(strength / STRENGTH_CONVERSION_FACTOR)
        hub.ble.broadcast(message_to_broadcast)
        print(stopwatch.time() - initial_time)
        wait(LOOP_DELAY_MS) # Delay
        
main()