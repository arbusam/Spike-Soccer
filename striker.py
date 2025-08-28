from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

# Shared motion helpers
from motion import move

# ---------------------------------------------
# Configuration constants — adjust as needed
# ---------------------------------------------
D_OFFSET                     = 0  # Compass correction (deg)
HIGH_STRENGTH                = 185  # Very strong IR signal
MED_STRENGTH                 = 130  # Moderate IR signal
LOW_STRENGTH                 = 120  # Weak IR signal
DIST_CLOSE                   = 25   # cm threshold for back-left obstacle
DIST_FAR                     = 90   # cm threshold for rear obstacle
MAX_SPEED                    = 1110 # Motor max speed
SLOW_SPEED                   = 300  # Backup / cautious speed
MEDIUM_SPEED                 = 700  # Lost speed
TOUCHING_SPEED               = 600  # Speed when touching ball
YAW_CORRECT_SLOWDOWN         = 50   # Slowdown for fast dynamic yaw correction (%)
YAW_CORRECT_SPEED            = 200  # Speed for fast dynamic yaw correction
YAW_CORRECT_THRESHOLD        = 15   # Fast dynamic yaw correction threshold
STATIC_YAW_CORRECT_THRESHOLD = 50   # Yaw correct threshold for static
STATIC_YAW_CORRECT_SPEED     = 100  # Static yaw correct speed
SLOW_YAW_CORRECT_SLOWDOWN    = 75   # Slowdown for slow dynamic yaw correction (%)
SLOW_YAW_CORRECT_SPEED       = 50   # Speed for slow dynamic yaw correction
SLOW_YAW_CORRECT_THRESHOLD   = 8    # Slow dynamic yaw correction threshold
LOOP_DELAY_MS                = 10   # Loop delay for cooperative multitasking
RIGHT_STEERING_THRESHOLD     = 100  # Threshold for right steering
LEFT_STEERING_THRESHOLD      = 80   # Threshold for left steering
STEERING_ANGULAR_DIRECTION   = 30   # The direction of steering in either direction
HOLDING_BALL_THRESHOLD       = 190  # Threshold after which the bot is considered to be 'holding' the ball
STRENGTH_CONVERSION_FACTOR   = 2.5  # Factor to convert striker strength to defence for communication

# --------------------------------------------
# Device initialization
# --------------------------------------------

a_motor = Motor(Port.E)
b_motor = Motor(Port.F)
c_motor = Motor(Port.C)
d_motor = Motor(Port.D)
hub = PrimeHub(observe_channels=[77], broadcast_channel=37)
ir_sensor = PUPDevice(Port.B)
us = UltrasonicSensor(Port.A)

# Set identical speed limits on all drive motors
a_motor.control.limits(MAX_SPEED)
b_motor.control.limits(MAX_SPEED)
c_motor.control.limits(MAX_SPEED)
d_motor.control.limits(MAX_SPEED)

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
    stop = False
    pressed = False
    finalDirection = 90
    message = None
    hub.imu.reset_heading(0)
    while True:
        # --- Stop Button ---
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

        # --- Static yaw correction ---
        yaw = hub.imu.heading("3D")
        yaw = ((yaw + 180) % 360) - 180
        if yaw > STATIC_YAW_CORRECT_THRESHOLD:
            hub.light.on(Color.RED)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(-STATIC_YAW_CORRECT_SPEED)
            continue
        elif yaw < -STATIC_YAW_CORRECT_THRESHOLD:
            hub.light.on(Color.RED)
            for motor in (a_motor, b_motor, c_motor, d_motor):
                motor.run(STATIC_YAW_CORRECT_SPEED)
            continue

        message = hub.ble.observe(77)
        message_to_broadcast = None

        dir, strength = Ir_Read_360_Sensor_Data(4)
        if dir == 0:
            hub.display.char("C")
            move(finalDirection, MEDIUM_SPEED, (d_motor, c_motor, b_motor, a_motor))
            hub.light.on(Color.VIOLET)
            continue
        # --- skip when no IR signal ---

        distance = us.distance() / 10

        if dir == 1:
            hub.display.number(1)
        elif dir == 2:
            hub.display.number(2)
        elif dir == 3:
            hub.display.number(3)
        elif dir == 4:
            hub.display.number(4)
        elif dir == 5:
            hub.display.number(5)
        elif dir == 6:
            hub.display.number(6)
        elif dir == 7:
            hub.display.number(7)
        elif dir == 8:
            hub.display.number(8)
        elif dir == 9:
            hub.display.number(9)
        elif dir == 10:
            hub.display.number(10)
        elif dir == 11:
            hub.display.number(11)
        elif dir == 12:
            hub.display.number(12)
        elif dir == 13:
            hub.display.number(13)
        elif dir == 14:
            hub.display.number(14)
        elif dir == 15:
            hub.display.number(15)
        elif dir == 16:
            hub.display.number(16)
        elif dir == 17:
            hub.display.number(17)
        elif dir == 18:
            hub.display.number(18)

        speed = MAX_SPEED
        if strength > HIGH_STRENGTH:
            speed = SLOW_SPEED
        elif strength == 0: #Go backwards
            speed = SLOW_SPEED
            finalDirection = 280
        #Forward Directional Commands
        if dir in (14, 15, 16) and strength >= HIGH_STRENGTH:
            if strength >= HOLDING_BALL_THRESHOLD:
                message_to_broadcast = "T"
                if distance > RIGHT_STEERING_THRESHOLD:
                    speed = TOUCHING_SPEED
                    finalDirection = STEERING_ANGULAR_DIRECTION
                    hub.display.char("R")
                elif distance < LEFT_STEERING_THRESHOLD:
                    speed = TOUCHING_SPEED
                    finalDirection = -STEERING_ANGULAR_DIRECTION
                    hub.display.char("L")
            finalDirection = 0
        elif dir == 14:# Forward
            finalDirection = 0
        elif dir == 15 and strength < HIGH_STRENGTH:
            finalDirection = 30
        elif dir == 13:
            finalDirection = 345
        elif dir == 8:
            finalDirection = 195
        elif dir == 10:
            finalDirection = 250
        elif dir == 11:
            finalDirection = 285
        elif dir == 12: #Double check
            finalDirection = 315
        elif dir == 16 and strength < HIGH_STRENGTH:
            finalDirection = 55
        elif dir == 17:# Front Right
            finalDirection = 90
        #Backwards Directional Commands
        elif dir == 5:
            finalDirection = 185
        elif dir == 6:
            finalDirection = 175
        elif dir == 7:
            finalDirection = 190
        elif dir == 18:
            finalDirection = 100
        elif dir == 4:# BackBackRight
            finalDirection = 180
        elif dir == 1:# BackRight
            finalDirection = 130
        elif dir == 3:
            finalDirection = 160
        #East-West Directional Commands
        elif dir == 2:# Right
            finalDirection = 170
        elif dir == 9:# Left
            finalDirection = 220
        finalDirection += D_OFFSET
        move(finalDirection, speed, (d_motor, c_motor, b_motor, a_motor))
        print([dir, speed, strength, finalDirection])
        if message_to_broadcast is None:
            hub.ble.broadcast(int(strength // STRENGTH_CONVERSION_FACTOR))
            print(strength, hub.ble.signal_strength(77))
        else:
            print(message_to_broadcast, hub.ble.signal_strength(77))
            hub.ble.broadcast(message_to_broadcast)
        wait(LOOP_DELAY_MS) # Delay
main()