from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# ---------------------------------------------
# Configuration constants — please don't touch
# ---------------------------------------------
HIGH_STRENGTH              = 65    # Very strong IR signal
MED_STRENGTH               = 60    # Moderate IR signal
LOW_STRENGTH               = 45    # Weak IR signal
DIST_TOUCHING              = 5     # cm threshold for touching obstacle
DIST_CLOSE                 = 25    # cm threshold for back-left obstacle
DIST_FAR                   = 90    # cm threshold for rear obstacle
MAX_SPEED                  = 1110  # Motor max speed
SLOW_SPEED                 = 500   # Backup / cautious speed
MED_SPEED                  = 800   # Medium speed
YAW_CORRECT_SPEED          = 1110  # Speed for yaw correction
YAW_CORRECT_THRESHOLD      = 15    # Yaw correction threshold
SLOW_YAW_CORRECT_THRESHOLD = 5     # Slow dynamic yaw correction threshold
SLOW_YAW_CORRECT_SPEED     = 100   # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_SLOWDOWN  = 90    # Slowdown for slow dynamic yaw correction (%)
LOOP_DELAY_MS              = 10   # Loop delay for cooperative multitasking
HOLDING_BALL_THRESHOLD     = 74    # Threshold after which the bot is considered to be 'holding' the ball
MIN_STRENGTH               = 5     # Minimum IR strength to consider a signal valid
TOUCHING_TIME_THRESHOLD    = 100   # ms threshold after which the bot is considered to be touching the ball
RIGHT_STEERING_THRESHOLD   = 100   # Threshold for right steering
LEFT_STEERING_THRESHOLD    = 80    # Threshold for left steering
HIGH_BLE_SIGNAL_THRESHOLD  = -50   # Threshold for high BLE signal strength to consider too close
LOW_BLE_SIGNAL_THRESHOLD   = -60   # Threshold for low BLE signal strength to consider too far

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
d_motor = Motor(Port.D)
hub = PrimeHub(observe_channels=[37], broadcast_channel=77)
us = UltrasonicSensor(Port.E)
ir_sensor = PUPDevice(Port.F)

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
    yaw = ((yaw + 180) % 360) - 180  # Normalize to [-180, 180)
    if yaw > SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far right, rotate left (dynamic)
        hub.light.on(Color.ORANGE)
        a_value = a_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
        b_value = b_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
        c_value = c_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
        d_value = d_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 + SLOW_YAW_CORRECT_SPEED
            

    elif yaw < -SLOW_YAW_CORRECT_THRESHOLD: # Rotated too far left, rotate right (dynamic)
        hub.light.on(Color.ORANGE)
        a_value = a_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
        b_value = b_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
        c_value = c_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED
        d_value = d_value * SLOW_YAW_CORRECT_SLOWDOWN // 100 - SLOW_YAW_CORRECT_SPEED

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
def main():
    stop = False
    pressed = False
    stopwatch = StopWatch()
    touchedTime = 0
    touching = False
    message = None
    yaw_correcting = False
    hub.imu.reset_heading(0)
    while True:
        data = hub.ble.observe(37)
        ble_signal = hub.ble.signal_strength(37)
        message = data
        striker_strength = -1
        if isinstance(message, int):
            striker_strength = message
            message = None
        message_to_broadcast: str | int = None

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
                motor.stop()
            continue

        # --- Static yaw correction ---
        yaw = hub.imu.heading("3D")
        yaw = ((yaw + 180) % 360) - 180
        if yaw > YAW_CORRECT_THRESHOLD:
            hub.ble.broadcast("Y")
            hub.light.on(Color.RED)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(YAW_CORRECT_SPEED)
            yaw_correcting = True
            continue
        elif yaw < -YAW_CORRECT_THRESHOLD:
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
        strength, ir = ir_sensor.read(5)[:2] # Read IR: strength, sector (1‑12 or 0)

        if strength < MIN_STRENGTH:
            ir = 0

        # --------------------
        # Check if signal exists
        # --------------------

        distance = us.distance() / 10

        if strength < HOLDING_BALL_THRESHOLD:
            touching = False
            if 1 <= ir <= 12:
                hub.display.number(ir)
            else:
                hub.display.char("C")
        if distance < 0:
            distance = 200
        if distance <= DIST_TOUCHING:
            if ir >= 11 or (ir >= 1 and ir <= 4):
                direction = 300
            else:
                direction = 240
            speed = SLOW_SPEED
        elif ir == 0:
            direction = 180
            speed = SLOW_SPEED
            # Reverse Steering
            if distance > RIGHT_STEERING_THRESHOLD:
                direction -= 40
            elif distance < LEFT_STEERING_THRESHOLD:
                direction += 40
        elif (message == "T" and ir in (1, 2, 3, 11, 12) and ble_signal > HIGH_BLE_SIGNAL_THRESHOLD) or (striker_strength != -1 and striker_strength > strength and ir in (1, 2, 3, 11, 12) and ble_signal > LOW_BLE_SIGNAL_THRESHOLD):
            speed = 0
        else:
            if ble_signal > LOW_BLE_SIGNAL_THRESHOLD and message == "T":
                speed = SLOW_SPEED
            else:
                speed = MAX_SPEED

            # --------------------
            # Heading decision
            # --------------------
            direction = ((ir-1) * 360 // 12)
        
            if ir > 5 and ir < 10:
                message_to_broadcast = "O"

            if message == "O":
                # TODO: Check ble signal strength. If high, stay back, if low move closer. However, if ball signal strength also high, hit ball.
                pass

            if ir == 1:
                if strength < HOLDING_BALL_THRESHOLD:
                    speed = MED_SPEED
                    direction = -10
                else:
                    message_to_broadcast = "T"
                    if not touching:
                        hub.display.number(1)
                        touching = True
                        touchedTime = stopwatch.time()
                        direction = -10
                    elif stopwatch.time() - touchedTime > TOUCHING_TIME_THRESHOLD:
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
                    direction = 0
                else: 
                    if not touching:
                        hub.display.number(2)
                        touching = True
                        touchedTime = stopwatch.time()
                        direction = 0 
                    elif stopwatch.time() - touchedTime > 500:
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
                speed = MED_SPEED
                direction = 75   # N for IR sector 2
            elif ir == 4 and strength >= MED_STRENGTH:
                direction = 150  # N for IR sector 39
            elif ir == 5 and strength >= MED_STRENGTH:
                direction = 225  # SW for IR sector 4
            elif ir == 6 and strength >= MED_STRENGTH:
                direction = 120
            elif ir == 7 and strength >= LOW_STRENGTH:
                if strength >= MED_STRENGTH:
                    direction = 90
                else:
                    direction = 120
            elif ir == 8 and strength >= LOW_STRENGTH:
                if strength >= MED_STRENGTH:
                    direction = 90
                else:
                    direction = 120
            elif ir == 9 and strength >= LOW_STRENGTH:
                direction = 180  # SSW for IR sector 8
            elif ir == 9 and strength >= HIGH_STRENGTH:
                direction = 145
            elif ir == 10 and strength >= MED_STRENGTH:
                direction = 200  # SSW for IR sector 9
            elif ir == 11 and strength >= HIGH_STRENGTH:
                direction = 200
            elif ir == 12:
                speed = MED_SPEED
                if strength >= HOLDING_BALL_THRESHOLD:
                    direction = 0
                # elif strength > MED_STRENGTH:
                #     direction = 250
                else:
                    direction = 300

            direction %= 360

        print(ir, direction, speed, strength, distance)
        move(direction, speed)
        print(ble_signal, message, striker_strength)
        if message_to_broadcast is None:
            message_to_broadcast = int(strength)
        hub.ble.broadcast(message_to_broadcast)
        wait(LOOP_DELAY_MS) # Delay

main()

