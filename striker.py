from hub import motion_sensor, port
import color_sensor, distance_sensor
import runloop
import motor

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET            = -10   # Compass correction (deg)
HIGH_STRENGTH       = 65    # Very strong IR signal
MED_STRENGTH        = 60    # Moderate IR signal
LOW_STRENGTH        = 45    # Weak IR signal
DIST_CLOSE          = 25    # cm threshold for front-right obstacle
DIST_FAR            = 90    # cm threshold for rear obstacle
MAX_SPEED           = 1110  # Motor max speed
SLOW_SPEED          = 500   # Backup / cautious speed
YAW_CORRECT_SPEED   = 700   # Speed for yaw correction
LOOP_DELAY_MS       = 10    # Loop delay for cooperative multitasking

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction, speed, yaw):
    """Drive holonomic robot toward `direction` with yaw correction."""
    if yaw > 100:   # Rotated too far right, rotate left
        for p in (port.C, port.D, port.A, port.B):
            motor.run(p, -YAW_CORRECT_SPEED)
        return
    if yaw < -100:  # Rotated too far left, rotate right
        for p in (port.C, port.D, port.A, port.B):
            motor.run(p, YAW_CORRECT_SPEED)
        return

    # Octant-based holonomic drive
    if 0 <= direction < 45:
        motor.run(port.C, -speed); motor.run(port.B, speed)
        motor.run(port.D, int((1 - direction/45) * speed))
        motor.run(port.A, int((direction/45 - 1) * speed))
    elif 45 <= direction < 90:
        motor.run(port.C, -speed); motor.run(port.B, speed)
        motor.run(port.D, int(-((direction-45)/45) * speed))
        motor.run(port.A, int(((direction-45)/45) * speed))
    elif 90 <= direction < 135:
        motor.run(port.D, -speed); motor.run(port.A, speed)
        motor.run(port.C, int(-((1 - (direction-90)/45)) * speed))
        motor.run(port.B, int((1 - (direction-90)/45) * speed))
    elif 135 <= direction < 180:
        motor.run(port.D, -speed); motor.run(port.A, speed)
        motor.run(port.C, int(((direction-135)/45) * speed))
        motor.run(port.B, int(-((direction-135)/45) * speed))
    elif 180 <= direction < 225:
        motor.run(port.C, speed); motor.run(port.B, -speed)
        motor.run(port.D, int(-((1 - (direction-180)/45)) * speed))
        motor.run(port.A, int((1 - (direction-180)/45) * speed))
    elif 225 <= direction < 270:
        motor.run(port.C, speed); motor.run(port.B, -speed)
        motor.run(port.D, int(((direction-225)/45) * speed))
        motor.run(port.A, int(-((direction-225)/45) * speed))
    elif 270 <= direction < 315:
        motor.run(port.D, speed); motor.run(port.A, -speed)
        motor.run(port.C, int((1 - ((direction-270)/45)) * speed))
        motor.run(port.B, int(-((direction-270)/45) * speed))
    elif 315 <= direction < 360:
        motor.run(port.D, speed); motor.run(port.A, -speed)
        motor.run(port.C, int(-((direction-315)/45) * speed))
        motor.run(port.B, int(((direction-315)/45) * speed))
    else:
        for p in (port.A, port.B, port.C, port.D):
            motor.stop(p)

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
async def main():
    while True:
        strength, ir = color_sensor.rgbi(port.F)[:2]  # Read IR: strength, sector (1-12 or 0)

        # Sector remap: 0→0, 1→12, n→n-1
        if ir:
            ir = 12 if ir == 1 else ir - 1

        # --------------------
        # Heading decision
        # --------------------
        if ir in (7, 8, 9, 2, 3) and strength >= HIGH_STRENGTH:
            direction = 160  # SSE
        elif ir == 4 and strength >= MED_STRENGTH:
            direction = 225  # SW
        elif ir == 6 and strength >= MED_STRENGTH:
            direction = 145 if distance_sensor.distance(port.E) > DIST_CLOSE else 270
        elif ir == 5 and strength >= LOW_STRENGTH:
            direction = 120 if distance_sensor.distance(port.E) > DIST_FAR else 240
        else:
            direction = int(360/12 * ir + D_OFFSET)

        direction %= 360

        # --------------------
        # Speed & reverse decision
        # --------------------
        if ir == 0:
            direction = 180  # south reverse when no signal
            speed = SLOW_SPEED
        else:
            speed = MAX_SPEED

        move(direction, speed, motion_sensor.tilt_angles()[0])
        await runloop.sleep_ms(LOOP_DELAY_MS)  # yield co-operative multitask

runloop.run(main())
