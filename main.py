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
    misc.reset_slider()

    # slap.left()
    # wait(1000)
    # slap.mid()
    # wait(1000)
    # slap.right()
    # wait(1000)
    while True:
        slap.left_block()
        slap.grab()
        slap.release()
        wait(2000)
        slap.right_block()
        slap.grab()
        slap.release()
        wait(2000)

    # Season Quest 4
    # # 1. Mold
    # acc.turn_degrees(-10, mode="spot")
    # acc.move_distance(300)
    # acc.turn_degrees(10, mode="spot")
    # acc.line_following(600)
    # acc.turn_degrees(90, mode="spot")
    # acc.move_distance(-70)
    # slap.slap_slapper()
    # acc.turn_degrees(-75, mode="spot")
    # acc.move_distance(240)
    # acc.turn_degrees(-15, mode="spot")
    # acc.line_following(180)
    # acc.turn_degrees(-90, mode="arc", turn_radius=240)
    # acc.turn_degrees(-110, mode="spot")
    # acc.turn_degrees(20, mode="spot")
    # slap.raise_slapper()
    # # # 2. Yellow
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(150, sensor=middleColor)
    # acc.line_following_blackvar()
    # acc.turn_degrees(90, mode="arc", turn_radius=280)
    # acc.line_following_blackvar(kp=0, kd=0)
    # acc.move_distance(-70)
    # slap.slap_slapper()
    # acc.move_distance(-180)
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(400, sensor=middleColor)
    # acc.turn_degrees(90, mode="arc", turn_radius=320)
    # # acc.line_following(380, sensor=rightColor)
    # # acc.turn_degrees(90, mode="spot")
    # acc.line_following(230, sensor=rightColor)
    # acc.turn_degrees(90, mode="arc", turn_radius=120)
    # # acc.line_following(130, sensor=middleColor)
    # acc.line_following(20, sensor=middleColor)
    # slap.raise_slapper()
    # # 3. Blue
    # acc.move_distance(-100)
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(650, sensor=leftColor)
    # acc.turn_degrees(-90, mode="spot")
    # acc.turn_degrees(90, mode="arc", turn_radius=210)
    # acc.line_following_blackvar()
    # acc.move_distance(-70)
    # slap.slap_slapper()
    # # # 4. Deviate for Flattener
    # acc.move_distance(-180)
    # acc.turn_degrees(135, mode="spot")
    # acc.move_distance(190)
    # acc.turn_degrees(45, mode="arc", turn_radius=200)
    # acc.line_following(200, sensor=rightColor)
    # acc.turn_degrees(-15, mode="spot")
    # acc.turn_degrees(15, mode="arc", turn_radius=200)
    # # 5. Finish Blue
    # acc.move_distance(-100)
    # acc.turn_degrees(90, mode="arc", turn_radius=400)
    # acc.move_distance(170)
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(150, sensor=middleColor)
    # slap.raise_slapper()
    # acc.move_distance(-170)
    # slap.slap_slapper()
    # acc.move_distance(90)


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
    # acc.line_following(200, sensor=leftColor)
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
    # # ending position facing mosaic
    # acc.line_following(350)
    # acc.line_following_blackvar()
    # acc.move_distance(160) # sensors at black mosaic start
    # #insert deposit code (motor C release, move forward 50mm~, motor C close, release, repeat)
    # # *code*
    # acc.move_distance(180) # to meet line after green deposit
    # acc.line_following(650)
    # acc.line_following_blackvar()
    # acc.move_distance(-20)
    # slap.slap_slapper() # green cement secured
    # acc.move_distance(-300)
    # acc.turn_degrees(180, mode="spot")
    # acc.line_following(360)
    # acc.line_following_blackvar()
    # slap.raise_slapper() # green cement released
    # acc.move_distance(-50)
    # acc.turn_degrees(180, mode="spot")

    # # double arc to go from green deposit to yellow cement (help values)
    # acc.turn_degrees(-90, mode="arc", turn_radius=250, default_max_speed=700)
    # acc.turn_degrees(90, mode="arc", turn_radius=270, default_max_speed=700)

    # acc.line_following(120)
    # acc.line_following_blackvar()
    # slap.slap_slapper() # yellow cement secured
    # acc.move_distance(-100)
    # acc.turn_degrees(180, mode="spot")
    # acc.turn_degrees(-90, mode="arc", turn_radius=250, default_max_speed=700)
    # acc.line_following(270)
    # acc.turn_degrees(90, mode="arc", turn_radius=270, default_max_speed=700)
    # acc.line_following(280) # right sensor
    # acc.turn_degrees(90, mode="arc", turn_radius=270, default_max_speed=700) # mini arc into yellow deposit
    # acc.line_following_blackvar()
    # slap.raise_slapper() # yellow cement released
    # acc.move_distance(-70)
    # acc.turn_degrees(180, mode="spot")
    # acc.turn_degrees(-90, mode="arc", turn_radius=250, default_max_speed=700) # mini arc to main line
    # acc.line_following(160) # left sensor (until middle sensor sees black?) 
    # acc.turn_degrees(90, mode="spot")
    # acc.line_following(60)
    # acc.line_following_blackvar()
    # slap.slap_slapper() # shovel secured
    # acc.turn_degrees(90, mode="arc", turn_radius=250, default_max_speed=700) # mini arc to face flattener
    # acc.move_distance(465) # push flattener to sponsor zone
    # acc.move_distance(-100)

    # # double arc to get back on main line (help values)
    # acc.turn_degrees(90, mode="arc", turn_radius=250, default_max_speed=700)
    # acc.turn_degrees(-90, mode="arc", turn_radius=270, default_max_speed=700)

    # acc.line_following(500) # idk which sensor to use so wont hit barrier or flattener
    # acc.turn_degrees(-45, mode="arc", turn_radius=270, default_max_speed=700) # put shovel into start zone
