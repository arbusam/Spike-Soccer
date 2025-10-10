from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.iodevices import PUPDevice

# ---------------------------------------------
# Configuration constants — please don't touch
# ---------------------------------------------
HIGH_STRENGTH                = 130    # Very strong IR signal
MED_STRENGTH                 = 100    # Moderate IR signal
LOW_STRENGTH                 = 90    # Weak IR signal
DIST_TOUCHING                = 5     # cm threshold for touching obstacle
DIST_CLOSE                   = 25    # cm threshold for back-left obstacle
DIST_FAR                     = 90    # cm threshold for rear obstacle
MAX_SPEED                    = 1110  # Motor max speed
SLOW_SPEED                   = 500   # Backup / cautious speed
SNAIL_SPEED                  = 100   # Very slow backup speed
MED_SPEED                    = 800   # Medium speed
YAW_CORRECT_SPEED            = 1110  # Speed for yaw correction
YAW_CORRECT_THRESHOLD        = 15    # Yaw correction threshold
SLOW_YAW_CORRECT_THRESHOLD   = 5     # Slow dynamic yaw correction threshold
SLOW_YAW_CORRECT_SPEED       = 100   # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_SLOWDOWN    = 10    # Slowdown for slow dynamic yaw correction (%)
LOOP_DELAY_MS                = 10    # Loop delay for cooperative multitasking
HOLDING_BALL_THRESHOLD       = 140   # Threshold after which the bot is considered to be 'holding' the ball
STRENGTH_CONVERSION_FACTOR   = 1     # Factor to align IR strength scale with striker communications
MIN_STRENGTH                 = 5     # Minimum IR strength to consider a signal valid
RIGHT_STEERING_THRESHOLD     = 100   # Threshold for right steering
LEFT_STEERING_THRESHOLD      = 80    # Threshold for left steering
HIGH_BLE_SIGNAL_THRESHOLD    = -40   # Threshold for high BLE signal strength to consider too close
LOW_BLE_SIGNAL_THRESHOLD     = -50   # Threshold for low BLE signal strength to consider too far
KICKOFF_TIME                 = 1000  # Amount of time (ms) to go forward when kicking off (left pressed while holding right)
MOVING_IR_LIST_LENGTH        = 5     # Length of list for moving average of IR strength
GOALIE_MAX_GOAL_DIST         = 50    # Maximum distance from goal to consider for goalie mode
GOALIE_MIN_GOAL_DIST         = 35    # Minimum distance from goal to consider for goalie mode
GOALIE_MAX_PUSH_DIST         = 100   # Maximum distance from goal to push out to
LOP_HAPPENING_STRENGTH       = 100   # Strength threshold to detect if ball is behind and is ready to hold

yaw_offset = 0

# Inputs: quadrant (0-3) and ratio (0-2)
# Quadrant: the sector of the full 360 degree circle in which the direction lies.
# Ratio: the position within that quadrant, where 0 is the start and 2 is the end.
# Outputs: a multiplier for each of the four motors (-1 to 1).

QUADRANT_FUNCS = [
    lambda r: (r-1, 1, -1, 1-r),    # 0°‑89° N → E
    lambda r: (1, 1-r, r-1, -1),    # 90°‑179° E → S
    lambda r: (1-r, -1, 1, r-1),    # 180°‑270° S → W
    lambda r: (-1, r-1, 1-r, 1),    # 270°‑359° W → N
]

# --------------------------------------------
# Device initialization
# --------------------------------------------

a_motor = Motor(Port.A)
b_motor = Motor(Port.B)
c_motor = Motor(Port.C)
d_motor = Motor(Port.F)
hub = PrimeHub(observe_channels=[37, 52], broadcast_channel=77)
us = UltrasonicSensor(Port.E)
ir_sensor = PUPDevice(Port.D)

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

a_motor.control.limits(MAX_SPEED)
b_motor.control.limits(MAX_SPEED)
c_motor.control.limits(MAX_SPEED)
d_motor.control.limits(MAX_SPEED)

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

    # --- Dynamic yaw correction ---
    yaw = hub.imu.heading("3D")
    yaw = (((yaw + 180) % 360) - 180) + yaw_offset # Normalize to [-180, 180)
    if yaw > SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far right, rotate left (dynamic)
        hub.light.on(Color.ORANGE)
        a_value = a_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 + SLOW_YAW_CORRECT_SPEED
        b_value = b_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 + SLOW_YAW_CORRECT_SPEED
        c_value = c_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 + SLOW_YAW_CORRECT_SPEED
        d_value = d_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 + SLOW_YAW_CORRECT_SPEED

    elif yaw < -SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right (dynamic)
        hub.light.on(Color.ORANGE)
        a_value = a_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 - SLOW_YAW_CORRECT_SPEED
        b_value = b_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 - SLOW_YAW_CORRECT_SPEED
        c_value = c_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 - SLOW_YAW_CORRECT_SPEED
        d_value = d_value * (100 - SLOW_YAW_CORRECT_SLOWDOWN) // 100 - SLOW_YAW_CORRECT_SPEED

    else:
        hub.light.off()

    # print(a_mult, b_mult, c_mult, d_mult, speed)
    a_motor.run(a_value)
    b_motor.run(b_value)
    c_motor.run(c_value)
    d_motor.run(d_value)


# ---------------------------------------------
# IR helper utilities
# ---------------------------------------------

def Ir_Combine_360_Sensor_Data(FrontDirection, FrontStrength, BackDirection, BackStrength):
    Direction, SignalStrength = 0, 0
    if FrontStrength == 0 and BackStrength == 0:
        Direction = 0
    else:
        if FrontStrength > BackStrength:
            Direction = round(FrontDirection) + 9
            SignalStrength = round(FrontStrength)
        else:
            Direction = round(BackDirection)
            SignalStrength = round(BackStrength)
    return Direction, SignalStrength


def Ir_Read_360_Sensor_Data(ReductionFactor):
    BackDirection = ir_sensor.read(1)[0]
    BackStrength, FrontStrength, FrontDirection = ir_sensor.read(5)[:3]
    return Ir_Combine_360_Sensor_Data(
        FrontDirection // ReductionFactor,
        FrontStrength // ReductionFactor,
        BackDirection // ReductionFactor,
        BackStrength // ReductionFactor,
    )

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
def main():
    stop = True
    right_pressed = False
    left_pressed = False
    bluetooth_pressed = False
    message = None
    yaw_correcting = False
    communication = True if hub.system.storage(0, read=1) == bytes([1]) else False
    hub.imu.reset_heading(0)
    goalie = True if hub.system.storage(1, read=1) == bytes([1]) else False
    active_setting = "GameMode"
    active_gamemode = "Goalie" if hub.system.storage(1, read=1) == bytes([1]) else "Defence"
    global yaw_offset
    ir = 0
    strlist = []
    while True:
        top_message = hub.ble.observe(52)
        distance = top_message / 10 if isinstance(top_message, int) else None
        if right_pressed:
            if Button.RIGHT not in hub.buttons.pressed() and top_message != "R":
                right_pressed = False
            else:
                hub.display.char("R")
                if Button.LEFT in hub.buttons.pressed() or top_message == "L":
                    hub.display.char("K")
                    move(0, MAX_SPEED)
                    wait(KICKOFF_TIME)
                continue
        elif Button.RIGHT in hub.buttons.pressed() or top_message == "R":
            stop = not stop
            right_pressed = True
            continue

        if stop:
            hub.ble.broadcast(None)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.stop()
            if left_pressed:
                if Button.LEFT not in hub.buttons.pressed() and top_message != "L":
                    left_pressed = False
                else:
                    continue
            elif Button.LEFT in hub.buttons.pressed() or top_message == "L":
                left_pressed = True
                if active_setting == "GameMode":
                    active_setting = "Communication"
                elif active_setting == "Communication":
                    active_setting = "GameMode"
            if bluetooth_pressed:
                if Button.BLUETOOTH not in hub.buttons.pressed() and top_message != "B":
                    bluetooth_pressed = False
                else:
                    continue
            elif (Button.BLUETOOTH in hub.buttons.pressed() or top_message == "B") and active_setting == "Communication" and bluetooth_pressed == False:
                communication = not communication
                bluetooth_pressed = True
                communication_bytes = bytes([1]) if communication else bytes([0])
                hub.system.storage(0, write=communication_bytes)
                continue
            elif (Button.BLUETOOTH in hub.buttons.pressed() or top_message == "B") and active_setting == "GameMode" and bluetooth_pressed == False:
                bluetooth_pressed = True
                if active_gamemode == "Goalie":
                    active_gamemode = "Defence"
                    goalie = False
                    hub.system.storage(1, write=bytes([0]))
                elif active_gamemode == "Defence":
                    active_gamemode = "Goalie"
                    goalie = True
                    hub.system.storage(1, write=bytes([1]))
                continue
            if active_setting == "GameMode":
                hub.light.on(Color.GREEN if active_gamemode == "Goalie" else Color.RED)
            else:
                hub.light.on(Color.GREEN if communication else Color.RED)
            hub.display.char("C" if active_setting == "Communication" else "M")
            continue
        ble_signal = None
        message = None
        striker_strength = -1
        striker_centred = False
        message_to_broadcast: str | int = None
        if communication:
            message = hub.ble.observe(37)
            ble_signal = hub.ble.signal_strength(37)
            striker_strength = -1
            if active_gamemode == "Goalie":
                if message == None:
                    goalie = False
                else:
                    goalie = True
            if isinstance(message, int):
                striker_strength = message
                if striker_strength < 0:
                    striker_strength = -striker_strength
                    striker_centred = True
                message = None
            elif message and message[0] == 'C':
                striker_centred = True
                message = message[1]
        skip_ir_logic = False

        direction = 0
        speed = MAX_SPEED

        # --- Static yaw correction ---
        yaw = hub.imu.heading("3D")
        yaw = (((yaw + 180) % 360) - 180) + yaw_offset
        if yaw > YAW_CORRECT_THRESHOLD:
            if communication:
                hub.ble.broadcast("Y")
            hub.light.on(Color.RED)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(YAW_CORRECT_SPEED)
            yaw_correcting = True
            continue
        elif yaw < -YAW_CORRECT_THRESHOLD:
            if communication:
                hub.ble.broadcast("Y")
            hub.light.on(Color.RED)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(-YAW_CORRECT_SPEED)
            yaw_correcting = True
            continue
        elif yaw_correcting:
            # Ensure the motors stop to avoid overcorrecting
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.hold()
            yaw_correcting = False
            continue

                # --- Read sensors ---
        ir, strength = Ir_Read_360_Sensor_Data(4)

        if strength < MIN_STRENGTH:
            ir = 0

         # --- Make Moving IR strength Values ---
        if len(strlist) < MOVING_IR_LIST_LENGTH:
            strlist.append(strength)

        elif len(strlist) == MOVING_IR_LIST_LENGTH:
            strlist.pop(0)
            strlist.append(strength)
        strength = sum(strlist) / len(strlist)
        back_distance = us.distance() / 10

        if goalie:
            message_to_broadcast = "G"
            direction = 0
            speed = MAX_SPEED
            print(back_distance, distance, ir, strength)
            if back_distance < GOALIE_MAX_PUSH_DIST and strength > HOLDING_BALL_THRESHOLD and ir in (14, 15):
                message_to_broadcast = "T"
                direction = 0
            elif distance is not None and distance < DIST_TOUCHING:
                direction = 300
            elif back_distance > GOALIE_MAX_GOAL_DIST:
                if (ir >= 2 and ir <= 7) and strength > LOP_HAPPENING_STRENGTH:
                    a_motor.hold()
                    b_motor.hold()
                    c_motor.hold()
                    d_motor.hold()
                    continue
                if (ir >= 16 or ir in (1, 2)):
                    direction = 135
                elif ir <= 13 and ir >= 9:
                    direction = 225
                else:
                    direction = 180
            elif back_distance < GOALIE_MIN_GOAL_DIST:
                if (ir >= 16 or ir in (1, 2)) and distance > DIST_CLOSE:
                    direction = 45
                elif ir <= 13 and ir >= 9:
                    direction = 315
                else:
                    direction = 0
            elif distance is not None and distance > RIGHT_STEERING_THRESHOLD:
                direction = 90
                speed = SLOW_SPEED
            elif distance is not None and distance < LEFT_STEERING_THRESHOLD:
                direction = 270
                speed = SLOW_SPEED
            elif ir >= 16 or (ir <= 4 and ir != 0):
                direction = 90
            elif ir <= 13 and ir >= 7:
                direction = 270
            elif ir == 0:
                if distance is not None and distance > RIGHT_STEERING_THRESHOLD:
                    direction = 90
                elif distance is not None and distance < LEFT_STEERING_THRESHOLD:
                    direction = 270
                else:
                    a_motor.hold()
                    b_motor.hold()
                    c_motor.hold()
                    d_motor.hold()
                    continue
            else:
                a_motor.hold()
                b_motor.hold()
                c_motor.hold()
                d_motor.hold()
                continue
            move(direction, speed)
            hub.ble.broadcast(message_to_broadcast)
            continue

        if ble_signal is not None and ble_signal > HIGH_BLE_SIGNAL_THRESHOLD and not ir in (1, 2):
            if striker_centred and distance > LEFT_STEERING_THRESHOLD and distance < RIGHT_STEERING_THRESHOLD:
                move(90, MED_SPEED)
            else:
                move(180, SLOW_SPEED)
            continue

        if distance <= DIST_TOUCHING:
            if ir >= 11 or ir == 1:
                direction = 300
            else:
                direction = 240
            speed = SLOW_SPEED
        elif ir == 0:
            message_to_broadcast = "L"
            direction = 180
            speed = SLOW_SPEED
            # Reverse Steering
            if distance > RIGHT_STEERING_THRESHOLD:
                direction -= 40
            elif distance < LEFT_STEERING_THRESHOLD:
                direction += 40
        elif (message == "T" and ir in (11, 12, 13, 14, 15, 16, 17, 18, 1) and ble_signal is not None and ble_signal > LOW_BLE_SIGNAL_THRESHOLD) or (striker_strength != -1 and striker_strength > strength and ir in (11, 12, 13, 14, 15, 16, 17, 18, 1) and ble_signal is not None and ble_signal > LOW_BLE_SIGNAL_THRESHOLD):
            speed = 0
        else:
            if ble_signal is not None and ble_signal > LOW_BLE_SIGNAL_THRESHOLD and message == "T":
                speed = SLOW_SPEED
            else:
                speed = MAX_SPEED

            # --------------------
            # Heading decision
            # --------------------
            direction = ((ir-1) * 360 // 12)
        
            if ir > 1 and ir < 11:
                message_to_broadcast = "O"

            if message == "O":
                # If ball is close, attack regardless.
                if strength >= MED_STRENGTH:
                    speed = MAX_SPEED
                else:
                    # Backwards steering to allow defence to own goal prevent.
                    if ble_signal is not None and ble_signal > HIGH_BLE_SIGNAL_THRESHOLD:
                        direction = 180
                        speed = SNAIL_SPEED
                        if distance > RIGHT_STEERING_THRESHOLD:
                            direction -= 40
                        elif distance < LEFT_STEERING_THRESHOLD:
                            direction += 40
                        skip_ir_logic = True
                    elif ble_signal is not None and ble_signal > LOW_BLE_SIGNAL_THRESHOLD:
                        speed = SLOW_SPEED
                    else:
                        speed = MED_SPEED

            if strength > HOLDING_BALL_THRESHOLD and ir in (13, 14, 15, 16) and not skip_ir_logic:
                message_to_broadcast = "T"
                if distance > RIGHT_STEERING_THRESHOLD:
                    direction = 30
                    hub.display.char("R")
                elif distance < LEFT_STEERING_THRESHOLD:
                    direction = 340
                    hub.display.char("L")
                else:
                    hub.display.number(14)
                    direction = 5
                    
            elif ir == 1 and not skip_ir_logic:
                direction = 100
            elif ir == 2 and strength < HIGH_STRENGTH and not skip_ir_logic:
                direction = 150
            elif ir == 2 and strength >= HIGH_STRENGTH and not skip_ir_logic:
                direction = 90
            elif ir == 3 and strength < HIGH_STRENGTH and not skip_ir_logic:
                direction = 170
            elif ir == 3 and strength >= HIGH_STRENGTH and not skip_ir_logic:
                direction = 90
            elif ir == 4 and strength < HIGH_STRENGTH and not skip_ir_logic:
                direction = 185
            elif ir == 4 and strength >= HIGH_STRENGTH and not skip_ir_logic:
                direction = 90
            elif ir == 5 and not skip_ir_logic:
                if strength >= HIGH_STRENGTH:
                    direction = 75
                elif strength >= MED_STRENGTH:
                    direction = 90
                else:
                    direction = 195
            elif ir == 6 and not skip_ir_logic:
                if strength >= HIGH_STRENGTH:
                    direction = 90
                else:
                    direction = 150
            elif ir == 7 and strength < HIGH_STRENGTH and not skip_ir_logic:
                direction = 200
            elif ir == 7 and strength >= HIGH_STRENGTH and not skip_ir_logic:
                direction = 145
            elif ir == 8 and strength < HIGH_STRENGTH and not skip_ir_logic:
                direction = 220
            elif ir == 8 and strength >= HIGH_STRENGTH and not skip_ir_logic:
                direction = 175
            # elif ir == 9 and not skip_ir_logic:
            #     if strength >= MED_STRENGTH:
            #         if distance < RAM_LEFT_STEERING_THRESHOLD:
            #             hub.display.char("L")
            #             direction = 270
            #         else:
            #             direction = 260
            #     else:
            #         direction = 260
            # elif ir == 10 and not skip_ir_logic:
            #     if strength >= MED_STRENGTH:
            #         if distance < RAM_LEFT_STEERING_THRESHOLD:
            #             hub.display.char("L")
            #             direction = 270
            #         else:
            #             direction = 265
            #     else:
            #         direction = 265
            elif ir == 9 and not skip_ir_logic:
                direction = 255
            elif ir == 10 and not skip_ir_logic:
                direction = 260
            elif ir == 11 and strength > HOLDING_BALL_THRESHOLD and not skip_ir_logic:
                direction = 180
            elif ir == 11 and not skip_ir_logic:
                direction = 310
            elif ir == 12 and strength > HOLDING_BALL_THRESHOLD and not skip_ir_logic:
                direction = 180
            elif ir == 12 and not skip_ir_logic:
                direction = 320
            elif ir == 13 and not skip_ir_logic:
                direction = 340
            elif ir == 14 and not skip_ir_logic:
                direction = 0
            elif ir == 15 and not skip_ir_logic:
                direction = 20
            elif ir == 16 and not skip_ir_logic:
                direction = 40
            elif ir == 17 and not skip_ir_logic:
                direction = 70
            elif ir == 18 and not skip_ir_logic:
                direction = 90

            direction %= 360

        print(ir, direction, speed, strength, distance, back_distance)
        move(direction, speed)
        if communication:
            # print(ble_signal, message, striker_strength, strength, direction)
            if message_to_broadcast is None:
                strength //= STRENGTH_CONVERSION_FACTOR
                message_to_broadcast = int(strength)
            hub.ble.broadcast(message_to_broadcast)
        wait(LOOP_DELAY_MS) # Delay

main()

