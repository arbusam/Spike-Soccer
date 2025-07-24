from hub import motion_sensor, port, button, light_matrix
import color_sensor, distance_sensor
import runloop
import motor

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
YAW_CORRECT_SPEED        = 700   # Speed for yaw correction
YAW_CORRECT_THRESHOLD    = 100   # Yaw correction threshold
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

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 90
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)

    motor.run(port.A, int(a_mult * speed))
    motor.run(port.B, int(b_mult * speed))
    motor.run(port.C, int(c_mult * speed))
    motor.run(port.D, int(d_mult * speed))

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
async def main():
    inverseOwnGoalPrevention = False
    stop = False
    pressed = False
    timer = 0
    touchedTime = 0
    touching = False
    while True:
        timer += LOOP_DELAY_MS
        if pressed:
            if button.pressed(button.RIGHT) == False:
                pressed = False
            else:
                continue
        elif button.pressed(button.RIGHT):
            stop = not stop
            pressed = True

        if stop:
            light_matrix.show_image(light_matrix.IMAGE_ASLEEP)
            for p in (port.A, port.B, port.C, port.D):
                motor.stop(p)
            continue
        continueInverseOwnGoalPrevention = False
        # --- Yaw emergency correction ---
        yaw = motion_sensor.tilt_angles()[0]
        if yaw > YAW_CORRECT_THRESHOLD:# Rotated too far right, rotate left
            for p in (port.A, port.B, port.C, port.D):
                motor.run(p, -YAW_CORRECT_SPEED)
            continue
        if yaw < -YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right
            for p in (port.A, port.B, port.C, port.D):
                motor.run(p, YAW_CORRECT_SPEED)
            continue

        # --- Read sensors ---
        strength, ir = color_sensor.rgbi(port.F)[:2]# Read IR: strength, sector (1‑12 or 0)

        if strength < MIN_STRENGTH:
            ir = 0

        # --------------------
        # Check if signal exists
        # --------------------

        distance = distance_sensor.distance(port.E) / 10

        if strength < HOLDING_BALL_THRESHOLD:
            touching = False
            if ir == 0:
                light_matrix.show_image(light_matrix.IMAGE_CONFUSED)
            elif ir == 1:
                light_matrix.write("1")
            elif ir == 2:
                light_matrix.write("2")
            elif ir == 3:
                light_matrix.write("3")
            elif ir == 4:
                light_matrix.write("4")
            elif ir == 5:
                light_matrix.write("5")
            elif ir == 6:
                light_matrix.write("6")
            elif ir == 7:
                light_matrix.write("7")
            elif ir == 8:
                light_matrix.write("8")
            elif ir == 9:
                light_matrix.write("9")
            elif ir == 10:
                light_matrix.write("+")
            elif ir == 11:
                light_matrix.write("-")
            elif ir == 12:
                light_matrix.write("=")
        elif touching:
            speed = SLOW_SPEED
        if distance < 0:
            distance = 200
        if distance <= DIST_TOUCHING:
            direction = 300
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
                        light_matrix.write("1")
                        touching = True
                        touchedTime = timer
                        direction = 5
                    elif timer - touchedTime > TOUCHING_TIME_THRESHOLD:
                        if distance > RIGHT_STEERING_THRESHOLD:
                            direction = 30
                            light_matrix.write("R")
                        elif distance < LEFT_STEERING_THRESHOLD:
                            direction = 340
                            light_matrix.write("L")
                        else:
                            light_matrix.write("1")
                            direction = 5
                    else:
                        light_matrix.write("1")
                        direction = 5
            elif ir == 2:
                if strength < HOLDING_BALL_THRESHOLD:
                    direction = 20
                else:
                    if not touching:
                        light_matrix.write("2")
                        touching = True
                        touchedTime = timer
                        direction = 20
                    elif timer - touchedTime > 500:
                        if distance > RIGHT_STEERING_THRESHOLD:
                            direction = 40
                            light_matrix.write("R")
                        elif distance < LEFT_STEERING_THRESHOLD:
                            direction = 350
                            light_matrix.write("L")
                        else:
                            light_matrix.write("2")
                            direction = 20
                    else:
                        light_matrix.write("2")
                        direction = 20
                    
            elif ir == 3 and strength >= HIGH_STRENGTH:
                direction = 180   # N for IR sector 2
            elif ir == 4 and strength >= HIGH_STRENGTH:
                direction = 180  # N for IR sector 3
            elif ir == 5 and strength >= LOW_STRENGTH:
                direction = 225  # SW for IR sector 4
            elif ir == 6 and strength >= LOW_STRENGTH:
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
                direction = 140  # SSW for IR sector 9
            elif ir == 11 and strength >= HIGH_STRENGTH:
                direction = 200
            elif ir == 12:
                if strength >= HOLDING_BALL_THRESHOLD:
                    direction = 180
                elif strength > MED_STRENGTH:
                    direction = 250
                else:
                    direction = 330

            direction %= 360
            if continueInverseOwnGoalPrevention:
                inverseOwnGoalPrevention = True
            else:
                inverseOwnGoalPrevention = False

        print(ir, direction, speed, strength, distance)
        move(direction, speed)
        await runloop.sleep_ms(LOOP_DELAY_MS) # Delay

runloop.run(main())

