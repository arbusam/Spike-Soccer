from hub import motion_sensor, port
import color_sensor, distance_sensor
import runloop
import motor

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET               = -10   # Compass correction (deg)
HIGH_STRENGTH          = 65    # Very strong IR signal
MED_STRENGTH           = 60    # Moderate IR signal
LOW_STRENGTH           = 45    # Weak IR signal
DIST_CLOSE             = 25    # cm threshold for back-left obstacle
DIST_FAR               = 90    # cm threshold for rear obstacle
MAX_SPEED              = 1110  # Motor max speed
SLOW_SPEED             = 500   # Backup / cautious speed
YAW_CORRECT_SPEED      = 700   # Speed for yaw correction
YAW_CORRECT_THRESHOLD  = 100   # Yaw correction threshold
LOOP_DELAY_MS          = 10    # Loop delay for cooperative multitasking

# Inputs: octant (0-7) and ratio (0-1)
# Octant: the sector of the full 360 degree circle in which the direction lies.
# Ratio: the position within that octant, where 0 is the start and 1 is the end.
# Outputs: a multiplier for each of the four motors (-1 to 1).

OCTANT_FUNCS = [
    lambda r: (r-1, 1, -1, 1-r),    # 0°‑44°N → NE
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
        # yaw = motion_sensor.tilt_angles()[0]
        # if yaw > YAW_CORRECT_THRESHOLD:  # Rotated too far right, rotate left
        #     for p in (port.A, port.B, port.C, port.D):
        #         motor.run(p, -YAW_CORRECT_SPEED)
        #     continue
        # if yaw < -YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right
        #     for p in (port.A, port.B, port.C, port.D):
        #         motor.run(p, YAW_CORRECT_SPEED)
        #     continue

        # --- Read sensors ---
        def Ir_Combine_360_Sensor_Data(FrontDirection, FrontStrength, BackDirection, BackStrength):   
            Direction, SignalStrength = 0, 0
            if (FrontStrength == 0 and BackStrength == 0):
                Direction = 0
            else:
                if (FrontStrength > BackStrength):
                    Direction = round(FrontDirection)
                    SignalStrength = round(FrontStrength)
                else:
                    Direction = round(BackDirection) + 9
                    SignalStrength = round(BackStrength)
            return Direction, SignalStrength

        def Ir_Read_360_Sensor_Data(Channel, ReductionFactor):
            rgb = color_sensor.rgbi(Channel)
            return Ir_Combine_360_Sensor_Data(color_sensor.reflection(Channel)//4, rgb[0]//ReductionFactor, rgb[2]//ReductionFactor, rgb[1]//ReductionFactor)
        dir, str = Ir_Read_360_Sensor_Data(port.F, 4)
        print([dir, str])
        speed = MAX_SPEED

        move(dir*20, speed)
        await runloop.sleep_ms(LOOP_DELAY_MS)  # Delay

runloop.run(main())