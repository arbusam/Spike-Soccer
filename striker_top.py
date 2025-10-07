from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

hub = PrimeHub(broadcast_channel=52, observe_channels=[37])

def main():
    while True:
        message_to_broadcast = ""
        if Button.LEFT in hub.buttons.pressed():
            message_to_broadcast += "L"
        if Button.RIGHT in hub.buttons.pressed():
            message_to_broadcast += "R"
        if Button.BLUETOOTH in hub.buttons.pressed():
            message_to_broadcast += "B"

        hub.ble.broadcast(message_to_broadcast)

main()
