#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)
line_sensor2 = ColorSensor(Port.S2)
robot = DriveBase(left_motor, right_motor, wheel_diameter = 55.5, axle_track = 104)
light = ColorSensor(Port.S1)
distance = 1000
reflection = 30
LPPK = 2
speed = 250

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 75

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
PROPORTIONAL_GAIN = 1.5

# Start following the line endlessly.
while robot.distance() >= distance:
    correction = (0 - reflection - light.reflection) * LPPK
    robot.drive(speed, correction)

robot.stop()
left_motor.brake()
right_motor.brake()


"""
# Kodi i fundit nga Zamiri
def follow_line_3stage():
    left_motor = Motor(Port.A, Direction.CLOCKWISE)
    right_motor = Motor(Port.B, Direction.CLOCKWEISE)
    line_sensor = ColorSenor(Port.S1)
    BLACK = 3
    WHITE = 62
    threshold_1 = int((WHITE - BLACK) / 3) + BLACK
    threshold_2 = WHITE-int((WHITE - BLACK) / 3)
    DRIVE_SPPED = 200

    while True:
        reflection_value = line_sensor.reflection
        if reflection_value <= threshold_1:
            right_motor.run_angle(DRIVE_SPEED, 50)
            left_motor.stop()
        elif reflection_value <= threshold_2:
            left_motor.run_angle(DRIVE_SPEED, 50)
            right_motor.stop()
        else:
            left_motor.run(DRIVE_SPEED)
            right_motor.run(DRIVE_SPEED)
            wait(10)


follow_line_3stage()"""

### programi i mire ############################################

"""ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter = 55.5, axle_track = 104)
light = ColorSensor(Port.S1)
distance = 1000
reflection = 30
LPPK = 2
speed = 250

while robot.distance() >= distance:
    correction = (0 - reflextion - light.reflection) * LPPK
    robot.drive(speed, correction)

robot.stop()
left_motor.brake()
right_motor.brake()"""