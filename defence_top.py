from pybricks.hubs import PrimeHub
from pybricks.pupdevices import UltrasonicSensor, ColorSensor
from pybricks.parameters import Button, Port, Color

hub = PrimeHub(broadcast_channel=52)

ultrasonic = UltrasonicSensor(Port.A)
# left = ColorSensor(Port.F)
# right = ColorSensor(Port.E)

# left.detectable_colors([Color()])
# right.detectable_colors([Color()])

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
        # print(left.hsv(True))

main()
