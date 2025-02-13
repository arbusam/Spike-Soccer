from hub import light_matrix, port, button
import runloop
import motor

async def main():
    moving = False
    direction = 115 # degrees, from 0-359
    speed = 1110 # from 0-1110
    while True:
        if (button.pressed(button.RIGHT)):
            moving = not moving
            while (button.pressed(button.RIGHT)):
                pass
        if (button.pressed(button.LEFT)):
            if (direction == 0):
                direction = 90
                light_matrix.write("R")
            elif (direction == 90):
                direction = 180
                light_matrix.write("B")
            elif (direction == 180):
                direction = 270
                light_matrix.write("L")
            elif (direction == 270):
                direction = 0
                light_matrix.write("F")
            else:
                direction = 0
                light_matrix.write("F")

            while (button.pressed(button.LEFT)):
                pass
        if (moving == True):
            if (direction >= 0 and direction < 45):
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

# TODO: If not moving, reverse

runloop.run(main())
