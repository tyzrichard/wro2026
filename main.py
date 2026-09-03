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
# motorD = Motor(Port.D)
leftColor = Ev3devSensor(Port.S1)
middleColor = Ev3devSensor(Port.S2)
rightColor = Ev3devSensor(Port.S3)
robot = DriveBase(motorA, motorB, wheel_diameter=62.4, axle_track=192)
print(ev3.battery.voltage())

acc = acceleration.AccelerationController(Kp=0.9)

if ev3.battery.voltage() >= 7000:
    # Code to run motor fast (for sanding)
    # motorC.run(-4000)
    # wait(1000000)

    # Code to reset slider, and move the slapper up and down repeatedly
    misc.reset_slider()
    motorC.run_target(900, -1500, then=Stop.HOLD)
    while True: # Is it is it a cart it's fine just shit I want to test on the color
        slap.slap_slapper()
        slap.raise_slapper()

    # # Season Quest 4
    # # 1. Mold
    # acc.turn_degrees(-5, mode="spot")
    # acc.move_distance(100)
    # acc.line_following(200)
    # acc.turn_degrees(90, mode="spot")
    # slap.slap_slapper()
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following(390)
    # acc.turn_degrees(-90, mode="arc", turn_radius=250)
    # acc.turn_degrees(-90, mode="spot")
    # slap.raise_slapper()
    # # 2. Yellow
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following_blackvar()
    # acc.turn_degrees(90, mode="arc", turn_radius=250)
    # acc.line_following_blackvar()
    # acc.move_distance(-70)
    # slap.slap_slapper()
    # acc.turn_degrees(180, mode="spot")
    # acc.line_following(180, sensor=middleColor)
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following(390, sensor=middleColor)
    # acc.turn_degrees(90, mode="arc", turn_radius=320)
    # acc.line_following(500, sensor=rightColor)
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(200, sensor=middleColor)
    # slap.raise_slapper()
    # # 3. Blue
    # acc.move_distance(-250)
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(200, sensor=leftColor)
    # acc.turn_degrees(-90, mode="arc", turn_radius=320)
    # acc.turn_degrees(90, mode="arc", turn_radius=320)
    # acc.line_following_blackvar()
    # acc.move_distance(-70)
    # slap.slap_slapper()
    # # 4. Deviate for Flattener
    # acc.turn_degrees(180, mode="spot")
    # acc.turn_degrees(-90, mode="arc", turn_radius=200)
    # acc.turn_degrees(90, mode="arc", turn_radius=200)
    # acc.line_following(300, sensor=rightColor)
    # acc.turn_degrees(-45, mode="spot")
    # acc.move_distance(100)
    # acc.turn_degrees(45, mode="spot")
    # acc.move_distance(300)
    # # 5. Finish Blue
    # acc.move_distance(-100)
    # acc.turn_degrees(45, mode="spot")
    # acc.move_distance(100)
    # acc.turn_degrees(-45, mode="spot")
    # acc.turn_degrees(180, mode="arc", turn_radius=250, default_max_speed=700)
    # # acc.turn_degrees(135, mode="arc", turn_radius=320)
    # acc.line_following_blackvar()
    # acc.move_distance(-70)
    # slap.raise_slapper()


# Season Quest 3
    # motorD.hold()
    # acc.turn_degrees(-90, mode="arc", turn_radius=250, default_max_speed=700)
    # acc.turn_degrees(90, mode="arc", turn_radius=270, default_max_speed=700)
    # while True:
    #     acc.line_following_blackvar()
    #     acc.move_distance(900)
    #     acc.line_following_blackvar()
    #     acc.turn_degrees(-90, mode="spot", default_max_speed=700)
    #     acc.turn_degrees(90, mode="arc", turn_radius=240)
    #     acc.line_following_blackvar(kp=0, kd=0)
    #     acc.turn_degrees(180, mode="spot")
    #     acc.line_following(250, sensor=middleColor)
    #     acc.turn_degrees(-90, mode="spot")
    #     acc.line_following(390, sensor=middleColor)
    #     acc.turn_degrees(90, mode="arc", turn_radius=320, default_max_speed=700)
    #     acc.line_following(800, sensor=rightColor)
    #     acc.turn_degrees(180, mode="arc", turn_radius=250, default_max_speed=700)
    

    # # #MAINS
    # acc.turn_degrees(30, mode="spot")
    # acc.move_distance(110)
    # acc.turn_degrees(-30, mode="spot")
    # acc.move_distance(100)
    # acc.line_following(200)
    # acc.turn_degrees(90, mode="spot")
    # slap.slap_slapper()
    # acc.turn_degrees(-90, mode="spot")
    # acc.line_following(390)
    # acc.turn_degrees(-90, mode="arc", turn_radius=250)
    # acc.turn_degrees(-90, mode="spot")
    # acc.move_distance(50)
    # slap.raise_slapper() # deposit bowl
    # acc.move_distance(-50)
    # acc.turn_degrees(90, mode="spot")
    # # acc.turn_degrees(-90, mode="arc", turn_radius=200) # help w this values to turn into aligning to run over mosaic
    # acc.line_following_blackvar()
    # # move forward scan colours
    # # *code*
    # # sorting algorithm
    # # *code*
