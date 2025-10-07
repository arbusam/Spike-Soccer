from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice

hub = PrimeHub(broadcast_channel=52, observe_channels=[37])

ultrasonic = UltrasonicSensor(Port.E)

def main():
    while True:
        if Button.LEFT in hub.buttons.pressed():
            hub.ble.broadcast("L")
        elif Button.RIGHT in hub.buttons.pressed():
            hub.ble.broadcast("R")
        elif Button.BLUETOOTH in hub.buttons.pressed():
            hub.ble.broadcast("B")
        else:
            hub.ble.broadcast(ultrasonic.distance())

main()
