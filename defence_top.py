from pybricks.hubs import PrimeHub
from pybricks.pupdevices import UltrasonicSensor
from pybricks.parameters import Button, Port

hub = PrimeHub(broadcast_channel=52)

ultrasonic = UltrasonicSensor(Port.D)

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
