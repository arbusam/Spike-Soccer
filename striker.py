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
DIST_CLOSE          = 25    # cm threshold for back-left obstacle
DIST_FAR            = 90    # cm threshold for rear obstacle
MAX_SPEED           = 1110  # Motor max speed
SLOW_SPEED          = 500   # Backup / cautious speed
YAW_CORRECT_SPEED   = 700   # Speed for yaw correction
LOOP_DELAY_MS       = 10    # Loop delay for cooperative multitasking

# Mapping of octant → function(r) returning speed multipliers for
# ports (A, B, C, D). r ∈ [0‑1] is progress through the octant.
OCTANT_FUNCS = [
    lambda r: (r-1, 1, -1, 1-r),    # 0°‑44°  N → NE
    lambda r: (r, 1, -1, -r),       # 45°‑89° NE → E
    lambda r: (1, 1-r, r-1, -1),    # 90°‑134° E → SE
    lambda r: (1, -r, r, -1),       # 135°‑179° SE → S
    lambda r: (1-r, -1, 1, r-1),    # 180°‑224° S → SW
    lambda r: (-r, -1, 1, r),       # 225°‑269° SW → W
    lambda r: (-1, -(1-r), 1-r, 1), # 270°‑314° W → NW
    lambda r: (-1, r, -r, 1)        # 315°‑359° NW → N
]

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 45
    ratio = (direction % 45) / 45
    a_mult, b_mult, c_mult, d_mult = OCTANT_FUNCS[octant](ratio)

    motor.run(port.A, int(a_mult * speed))
    motor.run(port.B, int(b_mult * speed))
    motor.run(port.C, int(c_mult * speed))
    motor.run(port.D, int(d_mult * speed))

# ---------------------------------------------
# Main control loop
# ---------------------------------------------
async def main():
    while True:
        # --- Yaw emergency correction ---
        yaw = motion_sensor.tilt_angles()[0]
        if yaw > 100:   # Rotated too far right, rotate left
            for p in (port.A, port.B, port.C, port.D):
                motor.run(p, -YAW_CORRECT_SPEED)
            continue
        if yaw < -100:  # Rotated too far left, rotate right
            for p in (port.A, port.B, port.C, port.D):
                motor.run(p, YAW_CORRECT_SPEED)
            continue

        # --- Read sensors ---
        strength, ir = color_sensor.rgbi(port.F)[:2]  # Read IR: strength, sector (1‑12 or 0)
        
        # Adjust IR to compensate for rotation of physical sensor
        if ir == 0:
            ir = -1
        else:
            ir = ir - 1
        
        # --------------------
        # Check if signal exists
        # --------------------
        if ir == -1:
            direction = 180  # south reverse when no signal
            speed = SLOW_SPEED
        else:
            speed = MAX_SPEED

            # --------------------
            # Heading decision
            # --------------------
            distance = distance_sensor.distance(port.E)
            if ir in (7, 8, 9, 2, 3) and strength >= HIGH_STRENGTH:
                direction = 160  # SSE
            elif ir == 4 and strength >= MED_STRENGTH:
                direction = 225  # SW
            elif ir == 6 and strength >= MED_STRENGTH:
                direction = 145 if distance > DIST_CLOSE else 270
            elif ir == 5 and strength >= LOW_STRENGTH:
                direction = 120 if distance > DIST_FAR else 240
            else:
                direction = (360 // 12) * ir + D_OFFSET

            direction %= 360

        move(direction, speed)
        await runloop.sleep_ms(LOOP_DELAY_MS)  # co‑operative multitask

runloop.run(main())
