from hub import light, motion_sensor, port, button, light_matrix, sound
import color
import color_sensor, distance_sensor
import runloop
import motor

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET            = -10# Compass correction (deg)
HIGH_STRENGTH        = 150    # Very strong IR signal
MED_STRENGTH        = 130    # Moderate IR signal
LOW_STRENGTH        = 120    # Weak IR signal
DIST_CLOSE            = 25    # cm threshold for back-left obstacle
DIST_FAR            = 90    # cm threshold for rear obstacle
MAX_SPEED            = 1110# Motor max speed
SLOW_SPEED            = 300# Backup / cautious speed
MEDIUM_SPEED         = 700 # Lost speed
YAW_CORRECT_SPEED    = 100# Speed for yaw correction
YAW_CORRECT_THRESHOLD = 75# Yaw correction threshold
LOOP_DELAY_MS        = 10    # Loop delay for cooperative multitasking

# Inputs: octant (0-7) and ratio (0-1)
# Octant: the sector of the full 360 degree circle in which the direction lies.
# Ratio: the position within that octant, where 0 is the start and 1 is the end.
# Outputs: a multiplier for each of the four motors (-1 to 1).

QUADRANT_FUNCS = [
    #E, F, C, D
    lambda r: (1, 1-r, r-1, -1),    # 0°‑89° N → E
    lambda r: (1-r, -1, 1, r-1),    # 90°‑179° E → S
    lambda r: (-1, r-1, 1-r, 1),    # 180°‑270° S → W
    lambda r: (r-1, 1, -1, 1-r),    # 270°‑359° W → N
]

# ---------------------------------------------
# Motor helper
# ---------------------------------------------

def move(direction: int, speed: int):
    """Drive robot toward `direction` (degrees) at `speed` (0-1110)."""

    # --- Lookup table for octant vectors ---
    octant = (direction % 360) // 90
    ratio = (direction % 90) / 90 * 2
    a_mult, b_mult, c_mult, d_mult = QUADRANT_FUNCS[octant](ratio)
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # --- Yaw emergency correction ---
    yaw = motion_sensor.tilt_angles()[0]
    if yaw > YAW_CORRECT_THRESHOLD:# Rotated too far right, rotate left
        sound.beep()
        light.color(light.POWER, color.RED)
        a_value += YAW_CORRECT_SPEED
        b_value += YAW_CORRECT_SPEED
        c_value += YAW_CORRECT_SPEED
        d_value += YAW_CORRECT_SPEED
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
        sound.beep()
        light.color(light.POWER, color.ORANGE)
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
        light.color(light.POWER, color.BLUE)

    # print(a_mult, b_mult, c_mult, d_mult, speed)
    motor.run(port.E, a_value)
    motor.run(port.F, b_value)
    motor.run(port.C, c_value)
    motor.run(port.D, d_value)

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
            Direction = round(FrontDirection)
            SignalStrength = round(FrontStrength)
        else:
            Direction = round(BackDirection) + 9
            SignalStrength = round(BackStrength)
    return Direction, SignalStrength

def Ir_Read_360_Sensor_Data(Channel, ReductionFactor):
    rgb = color_sensor.rgbi(Channel)
    return Ir_Combine_360_Sensor_Data(color_sensor.reflection(Channel)//4, rgb[0]//ReductionFactor, rgb[2]//ReductionFactor, rgb[1]//ReductionFactor)

async def main():
    stop = False
    pressed = False
    timer = 0
    finalDirection = 90
    while True:
        # --- Stop Button ---
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
            for p in (port.E, port.F, port.C, port.D):
                motor.stop(p)
            continue

        dir, str = Ir_Read_360_Sensor_Data(port.B, 4)
        if dir == 0:
            light_matrix.show_image(light_matrix.IMAGE_CONFUSED)
            move(finalDirection, MEDIUM_SPEED)
            continue
        finalDirection = (dir*20+9)%18
        finalDirection %= 360
        # --- skip when no IR signal ---

        distance = distance_sensor.distance(port.A) / 10

        if dir == 0:
            light_matrix.show_image(light_matrix.IMAGE_CONFUSED)
        elif dir == 1:
            light_matrix.write("1")
        elif dir == 2:
            light_matrix.write("2")
        elif dir == 3:
            light_matrix.write("3")
        elif dir == 4:
            light_matrix.write("4")
        elif dir == 5:
            light_matrix.write("5")
        elif dir == 6:
            light_matrix.write("6")
        elif dir == 7:
            light_matrix.write("7")
        elif dir == 8:
            light_matrix.write("8")
        elif dir == 9:
            light_matrix.write("9")
        elif dir == 10:
            light_matrix.write("+")
        elif dir == 11:
            light_matrix.write("-")
        elif dir == 12:
            light_matrix.write("=")
        elif dir == 13:
            light_matrix.write("S")
        elif dir == 14:
            light_matrix.write("Y")
        elif dir == 15:
            light_matrix.write("G")
        elif dir == 16:
            light_matrix.write("R")
        elif dir == 17:
            light_matrix.write("N")
        elif dir == 18:
            light_matrix.write("H")

        speed = MAX_SPEED
        if str > HIGH_STRENGTH:
            speed = SLOW_SPEED
        elif str == 0: #Go backwards
            speed = SLOW_SPEED
            finalDirection = 280
        #Forward Directional Commands
        if dir in (10, 11, 12, 13, 14, 15, 16, 17, 18) and str >= HIGH_STRENGTH:
            finalDirection = 90
        if dir in (13, 14, 15):# Forward
            finalDirection = 90
        elif dir == 8:
            finalDirection = 210
        elif dir == 10:
            finalDirection = 360
        elif dir == 11:
            finalDirection = 10
        elif dir == 12 and str < HIGH_STRENGTH:
            finalDirection = 30
        elif dir == 12 and str > HIGH_STRENGTH:
            finalDirection = 90
        elif dir == 16 and str < HIGH_STRENGTH:
            finalDirection = 180
        elif dir == 16 and str > HIGH_STRENGTH:
            finalDirection = 90
        elif dir == 17:# Front Right
            finalDirection = 200
        #Backwards Directional Commands
        elif dir in (5, 6, 7):# Backward
            finalDirection = 320
        elif dir == 18:
            finalDirection = 200
        elif dir == 4:# BackBackRight
            finalDirection = 320
        elif dir == 1:# BackRight
            finalDirection = 300
        #East-West Directional Commands
        elif dir == 2:# Right
            finalDirection = 280
        elif dir == 9:# Left
            finalDirection = 280
        move(finalDirection, speed)
        print([dir, speed, str, finalDirection])
        await runloop.sleep_ms(LOOP_DELAY_MS)# Delay

runloop.run(main())