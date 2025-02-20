from hub import motion_sensor, port, button
import color_sensor
import runloop
import motor

def move(direction, speed, yaw):
    if (yaw > 200):
        motor.run(port.A, -1110)
        motor.run(port.B, -1110)
        motor.run(port.C, -1110)
        motor.run(port.D, -1110)
    elif (yaw < -200):
        motor.run(port.A, 1110)
        motor.run(port.B, 1110)
        motor.run(port.C, 1110)
        motor.run(port.D, 1110)
    elif (direction >= 0 and direction < 45):
        motor.run(port.D, -speed)
        motor.run(port.C, speed)
        motor.run(port.B, int((1-(direction/45))*speed))
        motor.run(port.A, int((1-(direction/45))*-speed))
    elif (direction >= 45 and direction < 90):
        motor.run(port.D, -speed)
        motor.run(port.C, speed)
        motor.run(port.B, int(((direction-45)/45)*-speed))
        motor.run(port.A, int(((direction-45)/45)*speed))
    elif (direction >= 90 and direction < 135):
        motor.run(port.B, -speed)
        motor.run(port.A, speed)
        motor.run(port.D, int((1-((direction-90)/45))*-speed))
        motor.run(port.C, int((1-((direction-90)/45))*speed))
    elif (direction >= 135 and direction < 180):
        motor.run(port.B, -speed)
        motor.run(port.A, speed)
        motor.run(port.D, int(((direction-135)/45)*speed))
        motor.run(port.C, int(((direction-135)/45)*-speed))
    elif (direction >= 180 and direction < 225):
        motor.run(port.D, speed)
        motor.run(port.C, -speed)
        motor.run(port.B, int((1-(direction-180)/45)*-speed))
        motor.run(port.A, int((1-(direction-180)/45)*speed))
    elif (direction >= 225 and direction < 270):
        motor.run(port.D, speed)
        motor.run(port.C, -speed)
        motor.run(port.B, int(((direction-225)/45)*speed))
        motor.run(port.A, int(((direction-225)/45)*-speed))
    elif (direction >= 270 and direction < 315):
        motor.run(port.B, speed)
        motor.run(port.A, -speed)
        motor.run(port.D, int((1-((direction-270)/45))*speed))
        motor.run(port.C, int((1-((direction-270)/45))*-speed))
    elif (direction >= 315 and direction < 360):
        motor.run(port.B, speed)
        motor.run(port.A, -speed)
        motor.run(port.D, int(((direction-315)/45)*-speed))
        motor.run(port.C, int(((direction-315)/45)*speed))
    else:
        motor.stop(port.A)
        motor.stop(port.B)
        motor.stop(port.C)
        motor.stop(port.D)

async def main():
    direction = 115 # degrees, from 0-359
    speed = 1110 # from 0-1110
    while True:
        data = color_sensor.rgbi(port.E)
        irDirection = data[1]
        strength = data[0]

        if irDirection == 8 or irDirection == 9 or irDirection == 3 or irDirection == 4 and strength >= 65:
            direction = 180
        elif irDirection == 5 and strength >= 50:
            direction = 225
        elif irDirection == 7 and strength >= 50:
            direction = 145
        elif irDirection == 6 and strength >= 45:
            direction = 120
        else:
            direction = 360/12*irDirection
        if direction == 360:
            direction = 0
        elif direction == 0:
            direction = -1

        move(direction, speed, motion_sensor.tilt_angles()[0])

# TODO: If not moving, reverse

runloop.run(main())
