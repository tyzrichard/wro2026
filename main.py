from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from Hardware import motorA, motorB, motorC, motorD, robot, leftColor, middleColor, rightColor, pushButton
import Acceleration as acceleration
import NewAccel as new
import MiscSetup as misc
import SlapSorter as slap
import Logger as logger



ev3 = EV3Brick()

# motorA = Motor(Port.A)
# motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
# motorC = Motor(Port.C)
# motorD = Motor(Port.D)

# leftColor = color.HiTechnicColorV2(Port.S1)
# middleColor = color.HiTechnicColorV2(Port.S2)
# rightColor = color.HiTechnicColorV2(Port.S3)

# pushButton = TouchSensor(Port.S4)

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

    #ev3.speaker.beep()
    print("Touch button on Port 4 pressed")
    

if ev3.battery.voltage() >= 6900:
    try:
        # Testing: Print White Values of all 3 color sensors
        # while True:
        #     print(leftColor.read('RGB')[-1] + " " + middleColor.read('RGB')[-1] + " " + rightColor.read('RGB')[-1])
        
        # Testing: Move Slider
        # while True:
        #     slap.move("left_block")
        #     slap.slap_slapper()
        
        for i in range(10):
            wait_for_new_touch_press()
            wait(500)
            new.move_distance(500)
        # wait_for_new_touch_press()
        #ev3.speaker.beep(1000, 700)
        while True:
            wait_for_new_touch_press()
            print(logger.dump_log())

        # 0. Setup
        # ev3.speaker.beep()
        # wait_for_new_touch_press()
        # motorD.hold()
        # misc.reset_slider("right",False)
        # wait_for_new_touch_press()    

        # # 1. Move and get Yellows
        # slap.move("left_block")
        # new.move_distance(220)
        # new.turn_degrees(-90)
        # new.move_distance(270)
        # new.turn_degrees(-90)
        # acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
        # acc.line_following_blackvar()
        # new.move_distance(-25)
        # slap.grab()
        # slap.move("left")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release(wait=False)
        # new.move_distance(63)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("left")
        # slap.release(wait=False)
        # new.move_distance(64)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("left_block")
        # new.move_distance(-200)

        # # 2. Move and get Blues
        # new.turn_degrees(90)
        # new.move_distance(160) 
        # new.turn_degrees(-90)
        # acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
        # acc.line_following_blackvar(small=True)
        # new.move_distance(-27)
        # slap.grab()
        # slap.move("left")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release(wait=False)
        # new.move_distance(63)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("left")
        # slap.release(wait=False)
        # new.move_distance(64)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        
        # # 3. Throw all blocks onto mosaic and pray it works
        # new.move_distance(-200)
        # new.turn_degrees(150)
        # new.move_distance(100)
        # new.turn_degrees(30)
        # acc.line_following_blackvar()
        # slap.drop_blocks(1300)

        # # 4. Slap them greens
        # acc.line_following_blackvar()
        # new.move_distance(-70)
        # slap.slap_slapper()
        # new.turn_degrees(180)
        # new.move_distance(330)
        # slap.raise_slapper(wait=True)
        # new.move_distance(-170)
        # slap.slap_slapper()
        # new.move_distance(185)

        # # 5. ram the rest if got time
        # slap.raise_slapper(wait=True)
        # new.move_distance(-400)
        # acc.turn_degrees(-90, mode="arc", turn_radius=150, default_ramp_dist=100)
        # new.move_distance(270)
        # acc.turn_degrees(90, mode="arc", turn_radius=150, default_ramp_dist=100)
        # # slap.slap_slapper()
        # new.move_distance(750,default_ramp_dist=0)
        # # new.turn_degrees(25)
        # # new.move_distance(150)
        # # new.turn_degrees(-25)
        # # new.move_distance(200, default_ramp_dist=50)
    
    except Exception as e:
        logger.log_print(f"An error occurred: {e}")

        while True:
            wait_for_new_touch_press()
            print(logger.dump_log())
        
        raise