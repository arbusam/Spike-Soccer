from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Port

# ---------------------------------------------
# Configuration constants — please don't touch
# ---------------------------------------------
HIGH_STRENGTH            = 65    # Very strong IR signal
MED_STRENGTH             = 60    # Moderate IR signal
LOW_STRENGTH             = 45    # Weak IR signal
DIST_TOUCHING            = 5     # cm threshold for touching obstacle
DIST_CLOSE               = 25    # cm threshold for back-left obstacle
DIST_FAR                 = 90    # cm threshold for rear obstacle
MAX_SPEED                = 1110  # Motor max speed
SLOW_SPEED               = 500   # Backup / cautious speed
YAW_CORRECT_SPEED        = 1110   # Speed for yaw correction
YAW_CORRECT_THRESHOLD    = 15   # Yaw correction threshold
LOOP_DELAY_MS            = 10    # Loop delay for cooperative multitasking
HOLDING_BALL_THRESHOLD   = 74    # Threshold after which the bot is considered to be 'holding' the ball
MIN_STRENGTH             = 5     # Minimum IR strength to consider a signal valid
TOUCHING_TIME_THRESHOLD  = 100   # ms threshold after which the bot is considered to be touching the ball
RIGHT_STEERING_THRESHOLD = 100   # Threshold for right steering
LEFT_STEERING_THRESHOLD  = 80    # Threshold for left steering

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

# ---------------------------------------------
# Motor helper
# ---------------------------------------------
a_motor = Motor(Port.A)
b_motor = Motor(Port.B)
c_motor = Motor(Port.C)
d_motor = Motor(Port.D)
hub = PrimeHub()
ir_sensor = PUPDevice(Port.F)
us = UltrasonicSensor(Port.E)

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

    a_motor.run(int(a_mult * speed))
    b_motor.run(int(b_mult * speed))
    c_motor.run(int(c_mult * speed))
    d_motor.run(int(d_mult * speed))

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
def main():
    inverseOwnGoalPrevention = False
    stop = False
    pressed = False
    timer = 0
    touchedTime = 0
    touching = False
    yaw_correcting = False
    hub.imu.reset_heading(0)
    while True:
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
        continueInverseOwnGoalPrevention = False
        # --- Yaw emergency correction ---
        yaw = hub.imu.heading('3D')
        yaw = ((yaw + 180) % 360) - 180
        if yaw > YAW_CORRECT_THRESHOLD:# Rotated too far right, rotate left
            hub.display.char("Y")
            yaw_correcting = True
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(YAW_CORRECT_SPEED)
            continue
        elif yaw < -YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right
            hub.display.char("Y")
            yaw_correcting = True
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(-YAW_CORRECT_SPEED)
            continue
        elif yaw_correcting:
            yaw_correcting = False
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.hold()

        # --- Read sensors ---
        strength, ir = ir_sensor.read(5)[:2] # Read IR: strength, sector (1‑12 or 0)

        if strength < MIN_STRENGTH:
            ir = 0

        # --------------------
        # Check if signal exists
        # --------------------

        distance = us.distance() / 10

        if strength < HOLDING_BALL_THRESHOLD:
            touching = False
            if ir == 0:
                hub.display.char("C")
            elif ir == 1:
                hub.display.number(1)
            elif ir == 2:
                hub.display.number(2)
            elif ir == 3:
                hub.display.number(3)
            elif ir == 4:
                hub.display.number(4)
            elif ir == 5:
                hub.display.number(5)
            elif ir == 6:
                hub.display.number(6)
            elif ir == 7:
                hub.display.number(7)
            elif ir == 8:
                hub.display.number(8)
            elif ir == 9:
                hub.display.number(9)
            elif ir == 10:
                hub.display.number(10)
            elif ir == 11:
                hub.display.number(11)
            elif ir == 12:
                hub.display.number(12)
        if distance < 0:
            distance = 200
        if distance <= DIST_TOUCHING:
            if ir >= 11 or (ir >= 1 and ir <= 4):
                direction = 300
            else:
                direction = 240
            speed = SLOW_SPEED
        elif ir == 0:
            direction = 180 # south reverse when no signal
            speed = SLOW_SPEED
            # Reverse Steering
            if distance > RIGHT_STEERING_THRESHOLD:
                direction -= 40
            elif distance < LEFT_STEERING_THRESHOLD:
                direction += 40
        else:
            speed = MAX_SPEED

            # --------------------
            # Heading decision
            # --------------------
            direction = ((ir-1) * 360 // 12)

            if ir == 1:
                if strength < HOLDING_BALL_THRESHOLD:
                    direction = 0
                else:
                    if not touching:
                        hub.display.number(1)
                        touching = True
                        touchedTime = timer
                        direction = 5
                    elif timer - touchedTime > TOUCHING_TIME_THRESHOLD:
                        if distance > RIGHT_STEERING_THRESHOLD:
                            direction = 30
                            hub.display.char("R")
                        elif distance < LEFT_STEERING_THRESHOLD:
                            direction = 340
                            hub.display.char("L")
                        else:
                            hub.display.number(1)
                            direction = 5
                    else:
                        hub.display.number(1)
                        direction = 5
            elif ir == 2:
                if strength < HOLDING_BALL_THRESHOLD:
                    direction = 20
                else:
                    if not touching:
                        hub.display.number(2)
                        touching = True
                        touchedTime = timer
                        direction = 20
                    elif timer - touchedTime > 500:
                        if distance > RIGHT_STEERING_THRESHOLD:
                            direction = 40
                            hub.display.char("R")
                        elif distance < LEFT_STEERING_THRESHOLD:
                            direction = 350
                            hub.display.char("L")
                        else:
                            hub.display.number(2)
                            direction = 20
                    else:
                        hub.display.number(2)
                        direction = 30
                    
            elif ir == 3 and strength >= HIGH_STRENGTH:
                direction = 105   # N for IR sector 2
            elif ir == 4 and strength >= HIGH_STRENGTH:
                direction = 180  # N for IR sector 39
            elif ir == 5 and strength >= MED_STRENGTH:
                direction = 225  # SW for IR sector 4
            elif ir == 6 and strength >= MED_STRENGTH:
                if distance > DIST_FAR and not inverseOwnGoalPrevention:
                    direction = 120
                else:
                    continueInverseOwnGoalPrevention = True
                    direction = 240  # ESE/WSW for IR 5
            elif ir == 7 and strength >= LOW_STRENGTH:
                if distance > DIST_FAR and not inverseOwnGoalPrevention:
                    if strength >= HIGH_STRENGTH:
                        direction = 90
                    else:
                        direction = 120
                else:
                    continueInverseOwnGoalPrevention = True
                    if strength >= HIGH_STRENGTH:
                        direction = 270
                    else:
                        direction = 240  # ESE/WSW for IR 6
            elif ir == 8 and strength >= LOW_STRENGTH:
                if distance > DIST_FAR and not inverseOwnGoalPrevention:
                    if strength >= HIGH_STRENGTH:
                        direction = 90
                    else:
                        direction = 120
                else:
                    continueInverseOwnGoalPrevention = True
                    if strength >= HIGH_STRENGTH:
                        direction = 270
                    else:
                        direction = 240  # ESE/WSW for IR 7
            elif ir == 9 and strength >= HIGH_STRENGTH:
                direction = 140  # SSW for IR sector 8
            elif ir == 10 and strength >= HIGH_STRENGTH:
                direction = 200  # SSW for IR sector 9
            elif ir == 11 and strength >= HIGH_STRENGTH:
                direction = 200
            elif ir == 12:
                if strength >= HOLDING_BALL_THRESHOLD:
                    direction = 0
                elif strength > MED_STRENGTH:
                    direction = 250
                else:
                    direction = 330

            direction %= 360
            if continueInverseOwnGoalPrevention:
                inverseOwnGoalPrevention = True
            else:
                inverseOwnGoalPrevention = False

        # print(ir, direction, speed, strength, distance)
        move(direction, speed)
        wait(LOOP_DELAY_MS) # Delay

main()

