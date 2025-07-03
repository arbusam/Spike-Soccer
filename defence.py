from hub import motion_sensor, port, button
import color_sensor, distance_sensor
import runloop
import motor

# ---------------------------------------------
# Configuration constants — please don't touch
# ---------------------------------------------
HIGH_STRENGTH        = 65    # Very strong IR signal
MED_STRENGTH        = 60    # Moderate IR signal
LOW_STRENGTH        = 45    # Weak IR signal
DIST_TOUCHING       = 5     # cm threshold for touching obstacle
DIST_CLOSE            = 25    # cm threshold for back-left obstacle
DIST_FAR            = 90    # cm threshold for rear obstacle
MAX_SPEED            = 1110# Motor max speed
SLOW_SPEED            = 500# Backup / cautious speed
YAW_CORRECT_SPEED    = 700# Speed for yaw correction
YAW_CORRECT_THRESHOLD= 100# Yaw correction threshold
LOOP_DELAY_MS        = 10    # Loop delay for cooperative multitasking
HOLDING_BALL_THRESHOLD = 74    # Threshold after which the bot is considered to be 'holding' the ball

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
    while True:
        if pressed:
            if button.pressed(button.RIGHT) == False:
                pressed = False
            else:
                continue
        elif button.pressed(button.RIGHT):
            stop = not stop
            pressed = True

        if stop:
            for p in (port.A, port.B, port.C, port.D):
                motor.stop(p)
            continue
        newInverseOwnGoalPrevention = False
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

        if strength < 5:
            ir = 0

        # --------------------
        # Check if signal exists
        # --------------------

        distance = distance_sensor.distance(port.E) / 10
        if distance <= DIST_TOUCHING and distance > 0:
            direction = 300
            speed = SLOW_SPEED
        elif ir == 0:
            direction = 180 # south reverse when no signal
            speed = SLOW_SPEED
            if distance > 100:
                direction -= 40
            elif distance < 80:
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
                    if distance > 100:
                        direction = 40
                    elif distance < 80:
                        direction = 330
                    else:
                        direction = 10
            elif ir == 2:
                if strength < HOLDING_BALL_THRESHOLD:
                    direction = 10
                else:
                    if distance > 100:
                        direction = 50
                    elif distance < 80:
                        direction = 340
                    else:
                        direction = 10
            elif ir == 3 and strength >= HIGH_STRENGTH:
                direction = 135   # N for IR sector 2
            elif ir == 4 and strength >= HIGH_STRENGTH:
                direction = 170  # N for IR sector 3
            elif ir == 5 and strength >= MED_STRENGTH:
                direction = 225  # SW for IR sector 4
            elif ir == 6 and strength >= LOW_STRENGTH:
                if distance > DIST_FAR and not inverseOwnGoalPrevention:
                    direction = 120
                else:
                    newInverseOwnGoalPrevention = True
                    direction = 240  # ESE/WSW for IR 5
            elif ir == 7 and strength >= LOW_STRENGTH:
                if distance > DIST_FAR and not inverseOwnGoalPrevention:
                    if strength >= HIGH_STRENGTH:
                        direction = 90
                    else:
                        direction = 120
                else:
                    newInverseOwnGoalPrevention = True
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
                    newInverseOwnGoalPrevention = True
                    if strength >= HIGH_STRENGTH:
                        direction = 270
                    else:
                        direction = 240  # ESE/WSW for IR 7
            elif ir == 9 and strength >= HIGH_STRENGTH:
                direction = 200  # SSW for IR sector 8
            elif ir == 10 and strength >= HIGH_STRENGTH:
                direction = 200  # SSW for IR sector 9
            elif ir == 11:
                direction = 200
            elif ir == 12:
                if strength >= HOLDING_BALL_THRESHOLD:
                    direction = 180
                elif strength > HIGH_STRENGTH:
                    direction = 250
                else:
                    direction = 330

            direction %= 360
            if newInverseOwnGoalPrevention:
                inverseOwnGoalPrevention = True
            else:
                inverseOwnGoalPrevention = False

        print(direction, speed, strength, distance, ir)
        move(direction, speed)
        await runloop.sleep_ms(LOOP_DELAY_MS)# Delay

runloop.run(main())

