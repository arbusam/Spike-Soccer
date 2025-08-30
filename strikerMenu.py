from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.tools import hub_menu

hub = PrimeHub()

selected = hub_menu("G", "I")
hub.light.on(Color.CYAN)

if selected == "G":
    import striker
    striker.main()
elif selected == "I":
    import strikerIR
    strikerIR.main()