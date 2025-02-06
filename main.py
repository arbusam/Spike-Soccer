from hub import light_matrix, port, button
import runloop
import motor
# A forward
# B right
# C back
# D left

async def main():
    moving = False
    direction = 0 # 0: forwards, 1: right, 2: back, 3: left
    while True:
        if (button.pressed(button.RIGHT)):
            moving = not moving
            while (button.pressed(button.RIGHT)):
                pass
        if (button.pressed(button.LEFT)):
            direction += 1
            direction = direction % 4
            if (direction == 0):
                light_matrix.write("F")
            elif (direction == 1):
                light_matrix.write("R")
            elif (direction == 2):
                light_matrix.write("B")
            elif (direction == 3):
                light_matrix.write("L")
            while (button.pressed(button.LEFT)):
                pass
        if (moving == True):
            if (direction == 0):
                motor.run(port.A, -1110)
                motor.run(port.B, 1110)
                motor.run(port.C, 1110)
                motor.run(port.D, -1110)
            elif (direction == 1):
                motor.run(port.A, -1110)
                motor.run(port.B, -1110)
                motor.run(port.C, 1110)
                motor.run(port.D, 1110)
            elif (direction == 2):
                motor.run(port.A, 1110)
                motor.run(port.B, -1110)
                motor.run(port.C, -1110)
                motor.run(port.D, 1110)
            elif (direction == 3):
                motor.run(port.A, 1110)
                motor.run(port.B, 1110)
                motor.run(port.C, -1110)
                motor.run(port.D, -1110)
        else:
            motor.stop(port.A)
            motor.stop(port.B)
            motor.stop(port.C)
            motor.stop(port.D)

runloop.run(main())
