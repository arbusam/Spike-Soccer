from hub import motion_sensor, port
import color_sensor, distance_sensor
import runloop
import motor

# Controls robot motion based on IR beacon, distance sensor, and tilt

def move(direction, speed, yaw):
    # Control robot: heading 0-359°, speed, yaw tilt
    if (yaw > 100):
        # Rotated too far right, rotating left
        motor.run(port.C, -700)
        motor.run(port.D, -700)
        motor.run(port.A, -700)
        motor.run(port.B, -700)
    elif (yaw < -100):
        # Rotated too far left, rotating right
        motor.run(port.C, 700)
        motor.run(port.D, 700)
        motor.run(port.A, 700)
        motor.run(port.B, 700)
    elif (direction >= 0 and direction < 45):
        # Move NNE
        motor.run(port.C, -speed)
        motor.run(port.B, speed)
        motor.run(port.D, int((1-(direction/45))*speed))
        motor.run(port.A, int((1-(direction/45))*-speed))
    elif (direction >= 45 and direction < 90):
        # Move ENE
        motor.run(port.C, -speed)
        motor.run(port.B, speed)
        motor.run(port.D, int(((direction-45)/45)*-speed))
        motor.run(port.A, int(((direction-45)/45)*speed))
    elif (direction >= 90 and direction < 135):
        # Move ESE
        motor.run(port.D, -speed)
        motor.run(port.A, speed)
        motor.run(port.C, int((1-((direction-90)/45))*-speed))
        motor.run(port.B, int((1-((direction-90)/45))*speed))
    elif (direction >= 135 and direction < 180):
        # Move SSE
        motor.run(port.D, -speed)
        motor.run(port.A, speed)
        motor.run(port.C, int(((direction-135)/45)*speed))
        motor.run(port.B, int(((direction-135)/45)*-speed))
    elif (direction >= 180 and direction < 225):
        # Move SSW
        motor.run(port.C, speed)
        motor.run(port.B, -speed)
        motor.run(port.D, int((1-((direction-180)/45))*-speed))
        motor.run(port.A, int((1-((direction-180)/45))*speed))
    elif (direction >= 225 and direction < 270):
        # Move WSW
        motor.run(port.C, speed)
        motor.run(port.B, -speed)
        motor.run(port.D, int(((direction-225)/45)*speed))
        motor.run(port.A, int(((direction-225)/45)*-speed))
    elif (direction >= 270 and direction < 315):
        # Move WNW
        motor.run(port.D, speed)
        motor.run(port.A, -speed)
        motor.run(port.C, int((1-((direction-270)/45))*speed))
        motor.run(port.B, int((1-((direction-270)/45))*-speed))
    elif (direction >= 315 and direction < 360):
        # Move NNW
        motor.run(port.D, speed)
        motor.run(port.A, -speed)
        motor.run(port.C, int(((direction-315)/45)*-speed))
        motor.run(port.B, int(((direction-315)/45)*speed))
    else:
        # Stop
        motor.stop(port.A)
        motor.stop(port.D)
        motor.stop(port.B)
        motor.stop(port.C)

async def main():
    DIRECTION_OFFSET = -10
    direction = 0  # degrees, from 0-359
    speed = 1110   # from 0-1110
    while True:
        data = color_sensor.rgbi(port.F)  # Read IR beacon: (strength, direction)
        irDirection = data[1]
        strength = data[0]

        if (irDirection != 0):  # If irDirection != 0 do normalize sector
            irDirection -= 1
            if (irDirection == 0):  # If irDirection == 0 do wrap to 12
                irDirection = 12

        if (irDirection in (8, 9, 10, 3, 4) and strength >= 65):  # If obstacle sectors & high strength, head SSE (160°)
            direction = 160
        elif irDirection == 5 and strength >= 60:  # If front-left & high strength, head SW (225°)
            direction = 225
        elif irDirection == 7 and strength >= 60:  # If front-right & high strength, distance check
            if (distance_sensor.distance(port.E) > 25):  # If distance >25, head SE (145°)
                direction = 145
            else:  # Else head W (270°)
                direction = 270
        elif irDirection == 6 and strength >= 45:  # If rear & moderate strength, distance check
            if (distance_sensor.distance(port.E) > 90):  # If distance >90, head ESE (120°)
                direction = 120
            else:  # Else head WSW (240°)
                direction = 240
        else:  # Else default direction-based heading
            direction = 360/12*irDirection + DIRECTION_OFFSET

        if direction == 360:  # If direction ==360, reset to 0 & max speed
            direction = 0
            speed = 1110
        elif direction == 0:  # If direction ==0, reverse slow (180° S)
            direction = 180
            speed = 500
        else:  # Else full speed
            speed = 1110

        if irDirection == 0:  # If no IR signal, reverse slow (180° S)
            direction = 180
            speed = 500
        else:  # Else full speed
            speed = 1110

        direction %= 360
        move(direction, speed, motion_sensor.tilt_angles()[0])

runloop.run(main())
