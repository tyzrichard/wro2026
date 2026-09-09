from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

import Acceleration as acceleration
import MiscSetup as misc
import SlapSorter as slap

ev3 = EV3Brick()

motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)
motorD = Motor(Port.D)

leftColor = ColorSensor(Port.S1)
middleColor = ColorSensor(Port.S2)
rightColor = ColorSensor(Port.S3)

pushButton = TouchSensor(Port.S4)

robot = DriveBase(
    motorA,
    motorB,
    wheel_diameter=62.4,
    axle_track=192,
)

print(ev3.battery.voltage())

acc = acceleration.AccelerationController()

def touch_button_pressed():
    """Return True while the pushbutton is pressed."""
    return pushButton.pressed() 

def wait_for_new_touch_press():
    """Wait for one new press without counting a held button twice."""
    while touch_button_pressed():
        wait(10)

    while not touch_button_pressed():
        wait(10)

    ev3.speaker.beep()
    print("Touch button on Port 4 pressed")

if ev3.battery.voltage() >= 7000:
    # 0. Setup
    # ev3.speaker.beep()
    # wait(50)
    wait_for_new_touch_press()
    motorD.hold()
    misc.reset_slider("right",False)
    # while True:
    #     slap.move("left_block")
    #     slap.slap_slapper()
    wait_for_new_touch_press()    

    # 1. Move and get Yellows
    slap.move("left_block")
    acc.move_distance(220, default_ramp_dist=150)
    # acc.turn_degrees(-180, mode="arc", turn_radius=138, default_ramp_dist=150)
    acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=250)
    acc.move_distance(270, default_ramp_dist=150)
    acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=250)
    acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
    acc.line_following_blackvar(small=True)
    acc.move_distance(-25, default_ramp_dist=130)
    slap.grab()
    slap.move("left")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("mid")
    slap.release(wait=False)
    acc.move_distance(63, default_max_speed=600)
    slap.move("left_block")
    slap.grab()
    slap.move("right")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("left")
    slap.release(wait=False)
    acc.move_distance(64, default_max_speed=600)
    slap.move("left_block")
    slap.grab()
    slap.move("mid")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("right")
    slap.release()
    slap.move("left_block")
    acc.move_distance(-200)

    # 2. Move and get Blues
    acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=225)
    acc.move_distance(160, default_ramp_dist=250) 
    acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=225)
    acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
    acc.line_following_blackvar(small=True)
    acc.move_distance(-27, default_max_speed=600, default_ramp_dist=130)
    slap.grab()
    slap.move("left")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("mid")
    slap.release(wait=False)
    acc.move_distance(63, default_max_speed=600)
    slap.move("left_block")
    slap.grab()
    slap.move("right")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("left")
    slap.release(wait=False)
    acc.move_distance(64, default_max_speed=600)
    slap.move("left_block")
    slap.grab()
    slap.move("mid")
    slap.release()
    slap.move("right_block")
    slap.grab()
    slap.move("right")
    slap.release()
    acc.move_distance(30, default_max_speed=600)
    acc.turn_degrees(90, mode="spot", default_ramp_dist=200)
    acc.move_distance(360, default_ramp_dist=100)
    acc.move_distance(-240, default_ramp_dist=100)
    acc.turn_degrees(90, mode="spot", default_ramp_dist=400)
    

    # 3. Throw all blocks onto mosaic and pray it works
    # acc.turn_degrees(180, mode="spot")
    # acc.turn_degrees(-80, mode="arc", turn_radius=50, default_ramp_dist=160)
    # acc.turn_degrees(80, mode="arc", turn_radius=53, default_ramp_dist=160)
    acc.move_distance(300, default_ramp_dist=150)
    acc.line_following(350, default_ramp_dist=150, sensor=middleColor)
    acc.line_following_blackvar()
    slap.drop_blocks(1300)

    # 4. Slap them greens
    acc.line_following_blackvar()
    acc.move_distance(-70,default_ramp_dist=100)
    slap.slap_slapper()
    acc.turn_degrees(180, mode="spot", default_ramp_dist=175)
    acc.move_distance(330, default_ramp_dist=100)
    slap.raise_slapper(wait=True)
    acc.move_distance(-170, default_ramp_dist=100)
    slap.slap_slapper()
    acc.move_distance(185, default_ramp_dist=100)

    # 5. ram the rest if got time
    slap.raise_slapper(wait=True)
    acc.move_distance(-400)
    acc.turn_degrees(-90, mode="arc", turn_radius=150, default_ramp_dist=100)
    acc.move_distance(270, default_ramp_dist=100)
    acc.turn_degrees(90, mode="arc", turn_radius=150, default_ramp_dist=100)
    # slap.slap_slapper()
    acc.move_distance(750,default_ramp_dist=0)
    # acc.turn_degrees(25, mode="spot", default_ramp_dist=160)
    # acc.move_distance(150, default_ramp_dist=50)
    # acc.turn_degrees(-25, mode="spot", default_ramp_dist=160)
    # acc.move_distance(200, default_ramp_dist=50)