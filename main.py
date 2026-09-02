#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
import Acceleration as acceleration
import MiscSetup as misc
import SlapSorter as slap

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)
leftColor = Ev3devSensor(Port.S1)
middleColor = Ev3devSensor(Port.S2)
rightColor = Ev3devSensor(Port.S3)
robot = DriveBase(motorA, motorB, wheel_diameter=62.4, axle_track=192)
print(ev3.battery.voltage())

acc = acceleration.AccelerationController(Kp=0.9)

if ev3.battery.voltage() >= 7000:
    # misc.calibrate_sensor()
    # # slider weewoo
    # misc.reset_slider()
    # motorC.run_target(-500, -1000) # (speed, angle)
    # acc.blackstop()

    # # Phase 1: Move Bucket and Scan Mosaic
    # acc.move_distance(1000)
    # while True:
        # print(leftColor.read('RGB')[-1], rightColor.read('RGB')[-1])
    # acc.turn_degrees(-180, mode="arc")
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following(740)
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following(460)
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following_blackvar()
    


    # sensor_log = acc.move_colour_scan(600, default_max_speed=200)
    # for i in sensor_log: print(i)

    # Season Quest. We start from the yellow box facing towards it.
    
    acc.line_following(300, sensor=middleColor)
    acc.turn_degrees(-90, mode="spot")
    acc.line_following(400, sensor=middleColor)
    acc.turn_degrees(90, mode="arc", turn_radius=320)
    acc.line_following(800, sensor=rightColor)
    acc.turn_degrees(180, mode="arc", turn_radius=250)
    acc.line_following_blackvar()
    acc.move_distance(900)
    acc.line_following_blackvar()
    acc.turn_degrees(-90, mode="spot")
    acc.turn_degrees(90, mode="arc", turn_radius=210)
    acc.line_following_blackvar(kp=0, kd=0)
    acc.turn_degrees(180, mode="spot")
    acc.line_following(260, sensor=middleColor)
    acc.turn_degrees(-90, mode="spot")
    acc.line_following(400, sensor=middleColor)
    acc.turn_degrees(90, mode="arc", turn_radius=320)
    acc.line_following(800, sensor=rightColor)
    acc.turn_degrees(180, mode="arc", turn_radius=250)
    acc.line_following_blackvar()
    acc.move_distance(900)
    acc.line_following_blackvar()
    acc.turn_degrees(-90, mode="spot")
    acc.turn_degrees(90, mode="arc", turn_radius=210)
    acc.line_following_blackvar(kp=0, kd=0)
    acc.turn_degrees(180, mode="spot")




