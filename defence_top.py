from pybricks.hubs import PrimeHub
from pybricks.pupdevices import UltrasonicSensor
from pybricks.parameters import Button, Port

hub = PrimeHub(broadcast_channel=52)

ultrasonic = UltrasonicSensor(Port.A)

def main():
    while True:
        message_to_broadcast = ""
        if Button.LEFT in hub.buttons.pressed():
            message_to_broadcast += "L"
        if Button.RIGHT in hub.buttons.pressed():
            message_to_broadcast += "R"
        if Button.BLUETOOTH in hub.buttons.pressed():
            message_to_broadcast += "B"
        if message_to_broadcast == "":
            message_to_broadcast = ultrasonic.distance()
        
        distance = ultrasonic.distance() // 10
        if distance >= 100:
            distance == 0
        hub.display.number(distance)
        hub.ble.broadcast(message_to_broadcast)

main()
