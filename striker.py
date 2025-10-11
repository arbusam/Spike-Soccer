from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET                      = 5  # Compass correction (deg)
HIGH_STRENGTH                 = 170  # Very strong IR signal
MED_STRENGTH                  = 130  # Moderate IR signal
LOW_STRENGTH                  = 100  # Weak IR signal
DIST_CLOSE                    = 25   # cm threshold for back-left obstacle
DIST_FAR                      = 90   # cm threshold for rear obstacle
MAX_SPEED                     = 1000 # Motor max speed
MAX_ACCELERATION              = 3000 # Motor max acceleration
SLOW_SPEED                    = 300  # Backup / cautious speed
MEDIUM_SPEED                  = 350  # Lost speed
TOUCHING_SPEED                = 400  # Speed when touching ball
MAX_YAW_CORRECT_SLOWDOWN      = 20   # Slowdown for fast dynamic yaw correction (%)
MAX_YAW_CORRECT_SPEED         = 200  # Speed for fast dynamic yaw correction (Formula: YAW_CORRECT_SLOWDOWN% of MAX_SPEED should be > YAW_CORRECT_SPEED)
YAW_CORRECT_THRESHOLD         = 15   # Fast dynamic yaw correction threshold
STATIC_YAW_CORRECT_THRESHOLD  = 45   # Yaw correct threshold for static
STATIC_YAW_CORRECT_SPEED      = 500  # Static yaw correct speed
MAX_SLOW_YAW_CORRECT_SLOWDOWN = 1    # Slowdown for slow dynamic yaw correction (%)
MAX_SLOW_YAW_CORRECT_SPEED    = 1    # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_THRESHOLD    = 5    # Slow dynamic yaw correction threshold
LOOP_DELAY_MS                 = 10   # Loop delay for cooperative multitasking
RIGHT_STEERING_THRESHOLD      = 100  # Threshold for right steering
LEFT_STEERING_THRESHOLD       = 80   # Threshold for left steering
STEERING_ANGULAR_DIRECTION    = 5    # The direction of steering in either direction
HOLDING_BALL_THRESHOLD        = 200  # Threshold after which the bot is considered to be 'holding' the ball
STRENGTH_CONVERSION_FACTOR    = 1    # Factor to convert striker strength to defence for communication
KICKOFF_TIME                  = 1000 # Amount of time (ms) to go forward when kicking off (left pressed while holding right)
MOVING_IR_LIST_LENGTH         = 5    # Length of list for moving average of IR strength
HIGH_BLE_SIGNAL_THRESHOLD     = -40  # Threshold for high BLE signal strength to consider too close

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

a_motor = Motor(Port.A)
e_motor = Motor(Port.E)
c_motor = Motor(Port.C)
f_motor = Motor(Port.F)
hub = PrimeHub(observe_channels=[77], broadcast_channel=37)
ir_sensor = PUPDevice(Port.D)
us = UltrasonicSensor(Port.B)

a_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
e_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
c_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)
f_motor.control.limits(MAX_SPEED, MAX_ACCELERATION)

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for quadrant vectors ---
    quadrant = (direction % 360) // 90
    ratio = (direction % 90) / 45
    a_mult, e_mult, c_mult, f_mult = QUADRANT_FUNCS[quadrant](ratio)
    a_value = int(a_mult * speed)
    e_value = int(e_mult * speed)
    c_value = int(c_mult * speed)
    f_value = int(f_mult * speed)

    # --- Dynamic yaw correction ---
    yaw = hub.imu.heading("3D")
    yaw = ((yaw + 180) % 360) - 180  # Normalize to [-180, 180)
    abs_yaw = abs(yaw)
    if abs_yaw < SLOW_YAW_CORRECT_THRESHOLD:
        yaw_speed_mag = 0
        yaw_slowdown = 0
    elif abs_yaw < YAW_CORRECT_THRESHOLD:
        span = YAW_CORRECT_THRESHOLD - SLOW_YAW_CORRECT_THRESHOLD
        frac = (abs_yaw - SLOW_YAW_CORRECT_THRESHOLD) / span if span > 0 else 1
        yaw_speed_mag = frac * MAX_SLOW_YAW_CORRECT_SPEED
        yaw_slowdown = frac * MAX_SLOW_YAW_CORRECT_SLOWDOWN
    else:
        hub.light.on(Color.ORANGE)
        max_angle = STATIC_YAW_CORRECT_THRESHOLD
        span = max_angle - YAW_CORRECT_THRESHOLD
        capped_yaw = min(abs_yaw, max_angle)
        frac = (capped_yaw - YAW_CORRECT_THRESHOLD) / span if span > 0 else 1
        yaw_speed_mag = MAX_SLOW_YAW_CORRECT_SPEED + frac * MAX_YAW_CORRECT_SPEED
        yaw_slowdown = MAX_SLOW_YAW_CORRECT_SLOWDOWN + frac * MAX_YAW_CORRECT_SLOWDOWN

    if abs_yaw > 0:
        if yaw > 0:  # Rotated too far right, rotate left
            yaw_speed = -yaw_speed_mag
            slowdown = (100 - yaw_slowdown) / 100
            a_value = int(a_value * slowdown + yaw_speed)
            e_value = int(e_value * slowdown + yaw_speed)
            c_value = int(c_value * slowdown + yaw_speed)
            f_value = int(f_value * slowdown + yaw_speed)
        else:  # Rotated too far left, rotate right
            yaw_speed = yaw_speed_mag
            slowdown = (100 - yaw_slowdown) / 100
            a_value = int(a_value * slowdown + yaw_speed)
            e_value = int(e_value * slowdown + yaw_speed)
            c_value = int(c_value * slowdown + yaw_speed)
            f_value = int(f_value * slowdown + yaw_speed)
    else:
        hub.light.off()

    # print(a_mult, b_mult, c_mult, d_mult, speed)
    a_motor.run(a_value)
    e_motor.run(e_value)
    c_motor.run(c_value)
    f_motor.run(f_value)

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
    left_pressed = False
    bluetooth_pressed = False
    finalDirection = None
    message = None
    communication = True if hub.system.storage(0, read=1) == bytes([1]) else False
    hub.imu.reset_heading(0)
    stopwatch = StopWatch()
    strlist = []
    while True:
        if Button.BLUETOOTH in hub.buttons.pressed():
            if not bluetooth_pressed:
                bluetooth_pressed = True
        elif bluetooth_pressed:
            hub.display.char("K")
            stop = False
            kickoff_start_time = stopwatch.time()
            while stopwatch.time() - kickoff_start_time < KICKOFF_TIME:
                move(0, MAX_SPEED)
            bluetooth_pressed = False
        # --- Stop Button ---
        if pressed:
            if Button.RIGHT not in hub.buttons.pressed():
                pressed = False
            else:
                hub.display.char("R")
                hub.speaker.beep(64, 10)
                continue
        elif Button.RIGHT in hub.buttons.pressed():
            stop = not stop
            pressed = True
            continue

        if stop:
            hub.ble.broadcast(None)
            hub.light.on(Color.GREEN if communication else Color.RED)
            for motor in (a_motor, e_motor, c_motor, f_motor):
                motor.stop()
            if left_pressed:
                if Button.LEFT not in hub.buttons.pressed():
                    left_pressed = False
                else:
                    continue
            elif Button.LEFT in hub.buttons.pressed():
                communication = not communication
                left_pressed = True
                communication_bytes = bytes([1]) if communication else bytes([0])
                hub.system.storage(0, write=communication_bytes)
                hub.display.char("I" if communication else "O")
                continue
            hub.display.char("S")
            continue


        # --- Static yaw correction ---
        yaw = hub.imu.heading("3D")
        yaw = ((yaw + 180) % 360) - 180
        if abs(yaw) > STATIC_YAW_CORRECT_THRESHOLD:
            if communication:
                hub.ble.broadcast("Y")
            while abs(yaw) > YAW_CORRECT_THRESHOLD:
                if pressed:
                    if Button.RIGHT not in hub.buttons.pressed():
                        pressed = False
                elif Button.RIGHT in hub.buttons.pressed():
                    stop = not stop
                    pressed = True
                    break
                if yaw > STATIC_YAW_CORRECT_THRESHOLD:
                    hub.light.on(Color.RED)
                    for motor in (a_motor, e_motor, c_motor, f_motor):
                        motor.run(-STATIC_YAW_CORRECT_SPEED)
                elif yaw < -STATIC_YAW_CORRECT_THRESHOLD:
                    hub.light.on(Color.RED)
                    for motor in (a_motor, e_motor, c_motor, f_motor):
                        motor.run(STATIC_YAW_CORRECT_SPEED)
                yaw = hub.imu.heading("3D")
                yaw = ((yaw + 180) % 360) - 180
            for motor in (a_motor, e_motor, c_motor, f_motor):
                motor.hold()

        message = None
        defence_strength = -1
        ble_signal = None
        if communication:
            message = hub.ble.observe(77)
            ble_signal = hub.ble.signal_strength(77)
            defence_strength = -1
            if isinstance(message, int):
                defence_strength = message
                message = None
        message_to_broadcast = None

        # --- Read sensors ---
        dir, strength = Ir_Read_360_Sensor_Data(4)
        right_distance = us.distance() / 10

        # --- Make Moving IR strength Values ---
        if len(strlist) < MOVING_IR_LIST_LENGTH:
            strlist.append(strength)
        elif len(strlist) >= MOVING_IR_LIST_LENGTH:
            strlist.pop(0)
            strlist.append(strength)
        strength = sum(strlist) // len(strlist)

        if 1 <= dir <= 18:
            hub.display.number(dir)
        else:
            if dir == 0:
                if communication:
                    hub.ble.broadcast("L")
                hub.display.char("C")
                if message == "T" or (defence_strength != -1):
                    if right_distance > RIGHT_STEERING_THRESHOLD:
                        speed = SLOW_SPEED
                        finalDirection = 90
                    elif right_distance < LEFT_STEERING_THRESHOLD:
                        speed = SLOW_SPEED
                        finalDirection = -90
                    if finalDirection is None:
                        continue
                    move(finalDirection, MEDIUM_SPEED)
                    hub.light.on(Color.BLACK)
                    continue
                elif finalDirection is None:
                    continue
                else:
                    move(finalDirection, MEDIUM_SPEED)
                    hub.light.on(Color.VIOLET)
                    continue

        speed = MAX_SPEED
        if strength >= HIGH_STRENGTH:
            speed = MEDIUM_SPEED
        if message == "T" and dir not in (13, 14, 15) and ble_signal is not None and ble_signal > HIGH_BLE_SIGNAL_THRESHOLD:
            a_motor.stop()
            e_motor.stop()
            c_motor.stop()
            f_motor.stop()
            continue

        #Forward Directional Commands
        if dir == 14 and strength >= HOLDING_BALL_THRESHOLD:
            message_to_broadcast = "T"
            if right_distance > RIGHT_STEERING_THRESHOLD:
                speed = MAX_SPEED
                finalDirection = STEERING_ANGULAR_DIRECTION
                hub.display.char("R")
            elif right_distance < LEFT_STEERING_THRESHOLD:
                speed = MAX_SPEED
                finalDirection = -STEERING_ANGULAR_DIRECTION
                hub.display.char("L")
            else:
                finalDirection = 0
        elif dir in (1, 2, 3, 4, 5, 6, 7, 8) and strength >= HIGH_STRENGTH:
            speed = MEDIUM_SPEED
            finalDirection = 55
            message_to_broadcast = "O"
        elif dir == 1 and strength >= MED_STRENGTH:  # BackRight
            speed = MEDIUM_SPEED
            finalDirection = 150
            message_to_broadcast = "O"
        elif dir == 1:
            speed = MAX_SPEED
            finalDirection = 135
        elif dir == 2 and strength >= MED_STRENGTH:  # Right
            speed = MEDIUM_SPEED
            finalDirection = 170
            message_to_broadcast = "O"
        elif dir == 2:
            speed = MAX_SPEED
            finalDirection = 160
        elif dir == 3 and strength >= LOW_STRENGTH:
            speed = MEDIUM_SPEED
            finalDirection = 220
            message_to_broadcast = "O"
        elif dir == 3:
            speed = MAX_SPEED
            finalDirection = 170
        elif dir == 4 and strength >= LOW_STRENGTH:
            finalDirection = 250
            message_to_broadcast = "O"
        elif dir == 4:
            speed = MAX_SPEED
            finalDirection = 190
        elif dir == 5 and strength >= LOW_STRENGTH:
            finalDirection = 130
            message_to_broadcast = "O"
        elif dir == 5:
            speed = MAX_SPEED
            finalDirection = 180
        elif dir == 6 and strength >= MED_STRENGTH:
            finalDirection = 140
            message_to_broadcast = "O"
        elif dir == 6:
            speed = MAX_SPEED
            finalDirection = 180
        elif dir == 7 and strength >= MED_STRENGTH:
            finalDirection = 150
            message_to_broadcast = "O"
        elif dir == 7:
            speed = MAX_SPEED
            finalDirection = 235
        elif dir == 8 and strength >= MED_STRENGTH:
            finalDirection = 160
            message_to_broadcast = "O"
        elif dir == 8:
            speed = MAX_SPEED
            finalDirection = 215
        elif dir == 9 and strength >= MED_STRENGTH:  # Left
            speed = MEDIUM_SPEED
            finalDirection = 180
            message_to_broadcast = "O"
        elif dir == 9:
            speed = MAX_SPEED
            finalDirection = 260
        elif dir == 10 and strength >= LOW_STRENGTH:
            speed = MEDIUM_SPEED
            finalDirection = 200
            message_to_broadcast = "O"
        elif dir == 10:
            speed = MEDIUM_SPEED
            finalDirection = 270
        elif dir == 11:
            speed = MEDIUM_SPEED
            finalDirection = 240
        elif dir == 12 and strength >= MED_STRENGTH:
            finalDirection = 250
            speed = MEDIUM_SPEED
        elif dir == 12:
            finalDirection = 310
        elif dir == 13 and strength >= HOLDING_BALL_THRESHOLD:
            finalDirection = 0
            speed = MAX_SPEED
        elif dir == 13 and strength >= HIGH_STRENGTH:
            finalDirection = 345
            speed = MAX_SPEED
        elif dir == 13 and strength >= MED_STRENGTH:
            finalDirection = 325
            speed = MEDIUM_SPEED
        elif dir == 13:
            speed = MAX_SPEED
            finalDirection = 330
        elif dir == 14:  # Forward
            speed = MAX_SPEED
            finalDirection = 0
        elif dir == 15 and strength >= HIGH_STRENGTH:
            finalDirection = 10
            speed = MAX_SPEED
        elif dir == 15 and strength >= MED_STRENGTH:
            finalDirection = 10
            speed = MEDIUM_SPEED
        elif dir == 15:
            finalDirection = 10
        elif dir == 16 and strength >= MED_STRENGTH:
            finalDirection = 110
            speed = MEDIUM_SPEED
        elif dir == 16:
            finalDirection = 90
        elif dir == 17 and strength >= HIGH_STRENGTH:
            finalDirection = 180
        elif dir == 17 and strength >= MED_STRENGTH:
            finalDirection = 110
        elif dir == 17:  # Front Right
            speed = MEDIUM_SPEED
            finalDirection = 100
        elif dir == 18:
            speed = MEDIUM_SPEED
            finalDirection = 125
        if finalDirection is None:
            continue
        finalDirection += D_OFFSET
        move(finalDirection, speed)
        # print([dir, speed, strength, finalDirection])
        if message_to_broadcast is None:
            message_to_broadcast = int(strength // STRENGTH_CONVERSION_FACTOR)
            if right_distance > LEFT_STEERING_THRESHOLD and right_distance < RIGHT_STEERING_THRESHOLD:
                message_to_broadcast = -message_to_broadcast
        elif right_distance > LEFT_STEERING_THRESHOLD and right_distance < RIGHT_STEERING_THRESHOLD:
            message_to_broadcast = "C" + message_to_broadcast
        if communication:
            hub.ble.broadcast(message_to_broadcast)
        wait(LOOP_DELAY_MS) # Delay
        
try:
    main()
except Exception as e:
    hub.system.storage(1, write=bytes(str(e), 'utf-8'))
    raise