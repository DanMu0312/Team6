#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

left = Motor(Port.A)
right = Motor(Port.D)

robot = DriveBase(left,right,55.5,104)
cs1 = ColorSensor(Port.S1)
cs4 = ColorSensor(Port.S4)

threshold = 40
kp = 0.7




turn_table = [0,90,180,-90]

now_dir1 = 1
target_dir1 = 4
direction = (target_dir1 - now_dir1) %4
angle1 = turn_table[direction]

now_dir2 = 4
target_dir2 = 1
direction = (target_dir2 - now_dir2) %4
angle2 = turn_table[direction]

now_dir3 = 1
target_dir3 = 2
direction = (target_dir3 - now_dir3) %4
angle3 = turn_table[direction]

now_dir4 = 2
target_dir4 = 1
direction = (target_dir4 - now_dir4) %4
angle4 = turn_table[direction]
   

while True:
    left_reflection = cs1.reflection()
    right_reflection = cs4.reflection()

    if left_reflection < threshold:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp*error
        robot.drive(100,turn_rate)
    wait(10)   

  
    
robot.turn(angle1)

for i in range(2):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection < threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection > threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    
    
robot.turn(angle2)
for i in range(2):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection < threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection > threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    
    

robot.turn(angle3)



for i in range(1):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection < threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection > threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    
wait(100)

for i in range(1):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection < threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)    

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs4.reflection()

        if right_reflection > threshold:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)        
    
robot.turn(angle4)

while True:
    left_reflection = cs1.reflection()
    right_reflection = cs4.reflection()

    if left_reflection < threshold:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp * error
        robot.drive(100,turn_rate)
        wait(10)    
    
