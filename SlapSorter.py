#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
import Acceleration as acceleration
import MiscSetup as misc

motorC = Motor(Port.C)
motorD = Motor(Port.D)

def slap_slapper():
    # Runs until it physically can't move any further (hits the limit), then stops
    motorC.run_until_stalled(-1000, then=Stop.HOLD, duty_limit=90)

def raise_slapper():
    motorC.run_angle(600, 350) # speed, rot_angle

def grab():
    motorD.run_until_stalled(600, then=Stop.HOLD, duty_limit=90)

def release():
    motorD.run_until_stalled(-600, then=Stop.HOLD, duty_limit=90)

def left():
    motorC.run_target(1000, -100, then=Stop.HOLD)

def mid():
    motorC.run_target(1000, -650, then=Stop.HOLD)

def right():
    motorC.run_target(1000, -1250, then=Stop.HOLD)

def left_block():
    motorC.run_target(1000, -300, then=Stop.HOLD)

def right_block():
    motorC.run_target(1000, -1150, then=Stop.HOLD)

