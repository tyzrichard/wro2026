#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import MotorMovement as movement
import PDlinetracing as pd
import Acceleration as acc
import turning

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

def reset_slider():
    motorC.run(200)
    while not motorC.control.stalled():
        wait(10)
    motorC.stop()
    motorC.reset_angle(0)
    
accelControl = acc.AccelerationController(Kp=0.8)

if ev3.battery.voltage() >= 7000:
    # ARCHIVE: slider weewoo
    reset_slider()
    motorC.run_target(-500, -970) # (speed, angle)
    motorC.run_target(500, -800)
    motorC.run_target(-500, -970)
    motorC.run_target(500, -800)

    # Phase 1: Move Bucket and Scan Mosaic
    # accelControl.move_distance(850)
    # accelControl.turn_degrees(90, mode="spot")
    # accelControl.turn_degrees(-90, mode="spot")
    # accelControl.move_distance(750)
    # accelControl.turn_degrees(-90, mode="spot")
    # accelControl.move_distance(450)
    # accelControl.turn_degrees(-90, mode="spot")
    # accelControl.move_distance(600)

    # acc.line_following(940)
    # # turning.turn_degrees(motorA, motorB, angle_deg=90, mode="pivot")
    # # acc.line_following_blackvar(90)
    # # acc.line_following(-40, sensor=Port.S2)
    # # turning.turn_degrees(motorA, motorB, angle_deg=90, mode="pivot")
    # acc.synchronous_drive(300)

    # Phase 2: 


# while True:
#     error = leftColor.read('RGB')[-1] - threshold
#     dt = timer.time() / 1000.0
#     if dt == 0:
#         dt = 0.001
#     timer.reset()

#     derivative = (error - previous_error) / dt
#     steering = (kp * error) + (kd * derivative)
#     previous_error = error

#     robot.drive(drive_speed, steering)
#     wait(10)


# detectColor = "None"
# buffer = 30
# def checkColor(r, g, b, color):
#     if abs(color[0] - r) < buffer and abs(color[1] - g) < buffer and abs(color[2] - b) < buffer:
#         return True
#     return False

# while True:
#     colorReads = [leftColor.read('RGB'), middleColor.read('RGB'), rightColor.read('RGB')]
#     ev3.screen.clear()
#     # for color in colorReads:
#     #     if checkColor(255, 255, 255, color):
#     #         ev3.screen.print("White")
#     #     elif checkColor(60, 60, 60, color):
#     #         ev3.screen.print("Black")
#     #     elif checkColor(255, 255, 65, color):
#     #         ev3.screen.print("Yellow")
#     #     elif checkColor(65, 90, 150, color):
#     #         ev3.screen.print("Blue")
#     #     elif checkColor(75, 110, 65, color):
#     #         ev3.screen.print("Green")
#     #     else:
#     #         ev3.screen.print("NA")
#     # wait(200)


#     left_r, left_g, left_b, left_w = leftColor.read('RGB')
#     mid_r, mid_g, mid_b, mid_w = middleColor.read('RGB')
#     right_r, right_g, right_b, right_w = rightColor.read('RGB')

#     # Terminal output
#     print("LEFT: R={} G={} B={} | MID: R={} G={} B={} | RIGHT: R={} G={} B={}".format(
#         left_r, left_g, left_b, mid_r, mid_g, mid_b, right_r, right_g, right_b
#     ))

#     # EV3 Screen Display
#     ev3.screen.clear()
#     ev3.screen.print(" L  |  M  |  R ")
#     ev3.screen.print("R{:2d}|{:2d}|{:2d}".format(left_r, mid_r, right_r))
#     ev3.screen.print("G{:2d}|{:2d}|{:2d}".format(left_g, mid_g, right_g))
#     ev3.screen.print("B{:2d}|{:2d}|{:2d}".format(left_b, mid_b, right_b))
#     ev3.screen.print("W{:2d}|{:2d}|{:2d}".format(left_w, mid_w, right_w))
#     wait(200)

# def arc():
#     motorA.run_angle(750,625,wait=False)
#     motorB.run_angle(125,90,wait=True)
#     motorA.stop()
#     motorB.stop()
# def arcbig():
#     stopwatch = StopWatch()
#     motorA.run(-400)
#     motorB.run(-290)
#     stopwatch.reset()
#     while color.reflection() > 20 or stopwatch.time()< 3000:
#         pass
#     motorA.stop()
#     motorB.stop()
# def ccwspot_turn():
#     motorA.run_angle(500,250,wait=False)
#     motorB.run_angle(-500,250,wait=True)
#     motorA.stop()
#     motorB.stop()
# def cwspot_turn():
#     motorB.run_angle(500,250,wait=False)
#     motorA.run_angle(-500,250,wait=True)
#     motorA.stop()
#     motorB.stop()

# def phase1():
#     acc.acceleration_pd_line_following_phase1(time = 6.3)
#     motorA.run_angle(-100,7,wait=False)
#     motorB.run_angle(-100,7,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(250,-230,wait=True)
#     drop.stop()
#     wait(300)
#     spin.run_angle(180,180,wait=True)
#     spin.stop()
#     wait(300)
#     motorA.run_angle(-400,220,wait=False)
#     motorB.run_angle(-400,220,wait=True)
#     motorA.stop()
#     motorB.stop()
#     ccwspot_turn()
#     motorA.run_angle(600,950,wait=False)
#     motorB.run_angle(600,950,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(-500,135,wait=False)
#     motorB.run_angle(500,135,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     motorA.run_angle(600,270,wait=False)
#     motorB.run_angle(600,270,wait=True)
#     motorA.stop()
#     motorB.stop()
#     #drop red
#     drop.run_angle(400,230,wait=True)
#     drop.stop()
#     motorA.run_angle(-600,90,wait=False)
#     motorB.run_angle(-600,90,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(400,-230,wait=True)
#     drop.stop()
#     motorA.run_angle(-600,70,wait=False)
#     motorB.run_angle(-600,70,wait=True)
#     motorA.stop()
#     motorB.stop()
#     cwspot_turn()
#     motorA.run_angle(600,65,wait=False)
#     motorB.run_angle(600,65,wait=True)
#     drop.run_angle(400,230,wait=True)
#     drop.stop()
#     ccwspot_turn()

#     motorA.stop()
#     motorB.stop()
#     arcbig()
# def phase2():
#     global disc
#     global first_pair,second_pair
#     print(ev3.battery.voltage())
#     acc.pd_line_following_backwards(time=3.5, speed=170)
#     motorA.stop()
#     motorB.stop()
#     wait(100)
#     # motorA.run_angle(-300,1100,wait=False)
#     # motorB.run_angle(-300,1100,wait=True)
#     # motorA.stop()
#     # motorB.stop()
#     motorA.run_angle(-300,600,wait=False)
#     motorB.run_angle(-300,600,wait=True)
#     motorB.run_angle(-400,250,wait=False)
#     motorA.run_angle(-200,125,wait=True)
#     motorA.run_angle(-400,250,wait=False)
#     motorB.run_angle(-200,125,wait=True)
#     motorA.run_angle(-300,135,wait=False)
#     motorB.run_angle(-300,135,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(1000)

#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     color2 = color2[0]
#     if right.get(color2,0):
#         first_pair[1] = right.get(color2,first_pair[1])
#     else:
#         first_pair[1] = ""
#     #left then right
#     print(color, color2)
#     motorA.run_angle(-300,135,wait=False)
#     motorB.run_angle(-300,135,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     color2 = color2[0]
#     if right.get(color2,0):
#         first_pair[0] = right.get(color2,first_pair[0])
#     else:
#         first_pair[0] = ""
#     #left then right
#     print(color, color2)
#     motorA.run(-200)
#     motorB.run(-200)
#     wait(1500)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(300,200,wait=False)
#     motorB.run_angle(300,200,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     motorA.run_angle(400,400,wait=False)
#     motorB.run_angle(200,200,wait=True)
#     motorA.run_angle(200,200,wait=False)
#     motorB.run_angle(400,400,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     motorA.run_angle(-300,440,wait=False)
#     motorB.run_angle(-300,440,wait=True)
#     motorA.stop()
#     motorB.stop()
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     #left then right
#     print(color, color2)
#     color = color[0]
#     if left.get(color,0):
#         second_pair[0] = left.get(color,second_pair[0])
#     else:
#         second_pair[0] = ""
#     wait(300)
#     motorA.run_angle(-300,125,wait=False)
#     motorB.run_angle(-300,125,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     color = color[0]
#     if left.get(color,0):
#         second_pair[1] = left.get(color,second_pair[1])
#     else:
#         second_pair[1] = ""
#     #left then right
#     print(color, color2)
#     motorA.run(-200)
#     motorB.run(-200)
#     wait(1000)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     motorA.run_angle(300,400,wait=False)
#     motorB.run_angle(300,400,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorB.run_angle(470,470,wait=False)
#     motorA.run_angle(170,170,wait=True)
#     motorA.run_angle(470,470,wait=False)
#     motorB.run_angle(170,170,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(-300,60,wait=False)
#     motorB.run_angle(-300,60,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     #left then right
#     print(color, color2)
#     color2 = color2[0]
#     if right.get(color2):
#         disc = right.get(color2)
#     print(first_pair, second_pair, disc)
#     motorA.run_angle(300,20,wait=False)
#     motorB.run_angle(300,20,wait=True)
#     motorA.stop()
#     motorB.stop()
#     arc()
#     motorA.run_angle(300,100,wait=False)
#     motorB.run_angle(300,100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     arc()
#     motorA.stop()
#     motorB.stop()
#     motorA.run(200)
#     motorB.run(200)
#     color = ColorSensor(Port.S1)
#     color2 = ColorSensor(Port.S2)
#     while True:
#         if color.color() == Color.RED:
#             motorA.stop()
#             motorB.stop()
#             motorA.run_angle(300,70,wait=False)
#             motorB.run_angle(-300,70,wait=True)
#             motorA.stop()
#             motorB.stop()
#             wait(100)
#             break
#         if color2.color() == Color.RED:
#             motorA.stop()
#             motorB.stop()
#             motorA.run_angle(300,80,wait=False)
#             motorB.run_angle(-300,80,wait=True)
#             motorA.stop()
#             motorB.stop()
#             wait(100)
#             break


#     motorA.run_angle(-300,50,wait=False)
#     motorB.run_angle(-300,50,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,-230,wait=True)
#     motorA.run_angle(-300,240,wait=False)
#     motorB.run_angle(-300,240,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,230,wait=True)
# def phase3():
#     motorA.run_angle(-200,100,wait=False)
#     motorB.run_angle(-200,100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     stopwatch = StopWatch()
#     stopwatch.reset()
#     motorA.run(-500)
#     motorB.run(-500)
#     while stopwatch.time() <=1700:
#         pass
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(200,200,wait=False)
#     motorB.run_angle(200,200,wait=True)
#     motorA.stop()
#     motorB.stop()
#     ccwspot_turn()
#     acc.acceleration_pd_line_following(time=4.3)
#     acc.acceleration_pd_line_following_phase3(time=0.3,speed=200)
#     spin.run_target(200,0,wait=True)
#     motorA.run_angle(200,315,wait=False)
#     motorB.run_angle(200,315,wait=True)
#     motorA.stop()
#     motorB.stop()
#     cwspot_turn()
#     wait(300)
#     # motorA.run_angle(-200,23,wait=False)
#     # motorB.run_angle(200,23,wait=True)
#     # motorA.stop()
#     # motorB.stop()
#     # wait(300)
#     motorB.run(300)
#     motorA.run(300)
#     stopwatch = StopWatch()
#     stopwatch.reset()
#     while True:
#         if color.reflection() <= 30 and color2.reflection() <= 30 and stopwatch.time()>800:
#             break
#         if color.reflection() <= 30 and stopwatch.time()>800:
#             motorA.stop()
#         if color2.reflection() <= 30 and stopwatch.time()>800:
#             motorB.stop()
#     motorA.stop()
#     motorB.stop()
#     # motorA.run_angle(-200,5,wait=False)
#     # motorB.run_angle(-200,5,wait=True)
#     # motorA.stop()
#     # motorB.stop()
#     wait(300)
#     # motorA.run_angle(-200,20,wait=False)
#     # motorB.run_angle(-200,20,wait=True)
#     # motorA.stop()
#     # motorB.stop()
#     drop.run_angle(300,-230,wait=True)
#     wait(500)
#     motorB.run(-300)
#     motorA.run(-300)
#     stopwatch.reset()
#     while stopwatch.time() <=2000:
#         pass
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(400,1070,wait=False)
#     motorB.run_angle(400,1070,wait=True)
#     motorA.stop()
#     motorB.stop()
#     cwspot_turn()
#     motorB.run(-400)
#     motorA.run(-400)
#     stopwatch.reset()
#     while stopwatch.time() <=1500:
#         pass
#     rotate = {'g':-1,'b':0,'y':2,'r':1}
#     no = rotate.get(disc,0)
#     print(no)
#     print(no/abs(no) if no!=0 else 1)
#     sorting.rotate_all_motor(times = abs(no), direction = no/abs(no) if no!=0 else 1)
#     motorB.run(450)
#     motorA.run(450)
#     stopwatch.reset()
#     while True:
#         if color.reflection() <= 15 and color2.reflection() <= 15  and stopwatch.time() >=1000:
#             break
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(-200,10,wait=False)
#     motorB.run_angle(-200,10,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,230,wait=True)
# def phase2b():
#     global disc
#     global first_pair,second_pair
#     acc.pd_line_following_backwards(time=3.5, speed=170)
#     motorA.stop()
#     motorB.stop()
#     wait(100)
#     motorA.run_angle(-300,1100,wait=False)
#     motorB.run_angle(-300,1100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     color2 = color2[0]
#     if right.get(color2,0):
#         first_pair[1] = right.get(color2,0)
#     else:
#         first_pair[1] = ""
#     #left then right
#     print(color, color2)
#     motorA.run_angle(-300,135,wait=False)
#     motorB.run_angle(-300,135,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     color2= backright.read('COLOR')
#     color = backleft.read('COLOR')
#     color2 = color2[0]
#     if right.get(color2,0):
#         first_pair[0] = right.get(color2,0)
#     else:
#         first_pair[0] = ""
#     #left then right
#     print(color, color2)
# def phase4():
#     motorA.run_angle(-400,580,wait=False)
#     motorB.run_angle(-400,580,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(200)
#     ccwspot_turn()
#     motorA.run_angle(200,5,wait=False)
#     motorB.run_angle(-200,5,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     motorA.run_angle(200,190,wait=False)
#     motorB.run_angle(200,190,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(300)
#     drop.run_angle(600,-230,wait=True)
#     wait(1000)
#     motorA.run_angle(-200,70,wait=False)
#     motorB.run_angle(-200,70,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,230,wait=True)
#     motorA.run_angle(-100,100,wait=False)
#     motorB.run_angle(-480,480,wait=True)
#     motorA.run_angle(-480,480,wait=False)
#     motorB.run_angle(-100,100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(400,780,wait=False)
#     motorB.run_angle(400,780,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,-200,wait=True)
#     wait(1000)
#     motorA.run_angle(-200,250,wait=False)
#     motorB.run_angle(-200,250,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(300,200,wait=True)
#     cwspot_turn()
#     motorA.run(-500)
#     motorB.run(-500)
#     stopwatch = StopWatch()
#     stopwatch.reset()
#     spin.run_target(300,0)
#     while stopwatch.time() <= 3000:
#         pass
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(200,140,wait=False)
#     motorB.run_angle(200,140,wait=True)
#     motorA.stop()
#     motorB.stop()
#     ccwspot_turn()
#     motorA.run_angle(200,5,wait=False)
#     motorB.run_angle(-200,5,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorB.run(200)
#     motorA.run(200)
#     stopwatch = StopWatch()
#     stopwatch.reset()
#     while True:
#         if stopwatch.time() > 600:
#             if 15 <= color.reflection() <= 40 and 15 <= color2.reflection() <= 40:
#                 break   # reflections are in range, so stop
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(200,4,wait=False)
#     motorB.run_angle(200,4,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(400,-230,wait=True)
#     motorA.run_angle(200,50,wait=False)
#     motorB.run_angle(200,50,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(100)
#     motorA.run_angle(-200,50,wait=False)
#     motorB.run_angle(-200,50,wait=True)
#     motorA.stop()
#     motorB.stop()
#     fix_pairs()
#     sorting.sort_colors(first_pair,second_pair)
# def drop_house():
#     motorA.run_angle(-200,175,wait=False)
#     motorB.run_angle(-200,175,wait=True)
#     motorA.stop()
#     motorB.stop()
#     cwspot_turn()
#     motorA.run_angle(-200,5,wait=False)
#     motorB.run_angle(200,5,wait=True)
#     motorA.stop()
#     motorB.stop()

#     motorA.run(-300)
#     motorB.run(-300)
#     wait(1000)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(400,1230,wait=False)
#     motorB.run_angle(400,1230,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(500,500,wait=False)
#     motorB.run_angle(100,100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     spin.run_angle(-300,57,wait=False)
#     motorB.run_angle(100,70,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(200)
#     # motorA.run_angle(200,20,wait=False)
#     # motorB.run_angle(200,20,wait=True)
#     motorA.stop()
#     motorB.stop()
#     wait(200)
#     drop.run_angle(300,230,wait=True)
#     motorB.run_angle(-200,150,wait=True)
#     motorA.run_angle(200,13,wait=False)
#     motorB.run_angle(200,13,wait=True)
#     motorA.stop()
#     motorB.stop()
#     spin.run_angle(300,57,wait=True)
#     drop.run_angle(400,-230,wait=True)
#     motorA.run_angle(200,150,wait=True)
#     motorB.run_angle(200,130,wait=True)
#     motorA.run_angle(-400,400,wait=False)
#     motorB.run_angle(-200,200,wait=True)
#     motorA.run_angle(-400,1040,wait=False)
#     spin.run_angle(300,90,wait=False)
#     motorB.run_angle(-400,1040,wait=True)
#     motorA.stop()
#     motorB.stop()
#     spin.stop()
#     motorA.run_angle(200,200,wait=False)
#     motorB.run_angle(550,550,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(400,570,wait=False)
#     motorB.run_angle(400,570,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(300,170,wait=True)
#     motorB.run_angle(300,160,wait=True)
#     motorA.stop()
#     motorA.run(500)
#     motorB.run(500)
#     wait(2300)
#     while True:
#         if color.reflection() <= 10 and color2.reflection() <= 10:
#             break   # reflections are in range, so stop
#     motorA.stop() 
#     motorB.stop()
#     motorA.run_angle(-500,250,wait=False)
#     motorB.run_angle(500,250,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run(-600)
#     motorB.run(-600)
#     wait(3500)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(200,100,wait=False)
#     motorB.run_angle(200,100,wait=True)
#     motorA.stop()
#     motorB.stop()
#     motorA.run_angle(-500,250,wait=False)
#     motorB.run_angle(500,250,wait=True)
#     motorA.stop()
#     motorB.stop()
#     drop.run_angle(400,230,wait=True)
#     motorA.run(300)
#     motorB.run(300)
#     while True:
#         if color.reflection() <= 10 and color2.reflection() <= 10:
#             break   # reflections are in range, so stop
#     motorA.stop() 
#     motorB.stop()
#     motorA.run_angle(300,75,wait=False)
#     motorB.run_angle(300,75,wait=True)
#     motorA.stop()
#     motorB.stop()
#     # motorB.run_angle(200,30,wait=True)
#     # motorA.stop()
#     # motorB.stop()
#     # wait(200)
# # phase1()
# # motorA.run_angle(-500,190,wait=False)
# # motorB.run_angle(500,190,wait=True)
# # motorA.stop()
# # motorB.stop()
# # motorA.run_angle(-500,100,wait=False)
# # motorB.run_angle(-500,100,wait=True)
# # motorA.stop()
# # motorB.stop()
# # phase2()
# # phase3()
# # phase2b()

# #starts here
# # acc.acceleration_pd_line_following_phase1(time = 0.5,speed=800)
# # wait(100)
# # motorB.run_angle(-500,5,wait=False)
# # motorA.run_angle(-500,5,wait=True)
# # motorA.stop()
# # motorB.stop()
# # wait(100)
# # motorA.run_angle(500,254,wait=False) # can change this to 262 if battery low
# # motorB.run_angle(-500,254,wait=True)
# # motorA.stop()
# # motorB.stop()
# # wait(300)
# # motorA.run_angle(500,1500,wait=False)
# # motorB.run_angle(500,1500,wait=True)

# # motorA.stop()
# # motorB.stop()
# # color2= backright.read('COLOR')
# # color = backleft.read('COLOR')
# # print(color, color2)
# # first_pair[1] = left.get(color[0],first_pair[1])
# # second_pair[0] = right.get(color2[0],second_pair[0] )
# # motorA.run_angle(300,155,wait=False)
# # motorB.run_angle(300,155,wait=True)
# # motorA.stop()
# # motorB.stop()
# # color2= backright.read('COLOR')
# # color = backleft.read('COLOR')
# # print(color, color2)
# # first_pair[0] = left.get(color[0],first_pair[0])
# # second_pair[1] = right.get(color2[0],second_pair[1])
# # print(first_pair,second_pair)
# # motorB.run_angle(-500,500,wait=False)
# # motorA.run_angle(-500,500,wait=True)
# # motorA.stop()
# # motorB.stop()
# # wait(100)
# # motorB.run_angle(-150,150,wait=False)
# # motorA.run_angle(-340,340,wait=True)
# # wait(100)
# # motorA.run_angle(-150,150,wait=False)
# # motorB.run_angle(-340,340,wait=True)
# # motorA.stop()
# # motorB.stop()
# # wait(200)
# # motorA.run_angle(500,250,wait=False)
# # motorB.run_angle(500,250,wait=True)
# # motorA.stop()
# # motorB.stop()
# # wait(100)
# # color = backleft.read('COLOR')
# # print(color)
# # disc = left.get(color[0],"")
# # print(disc)
# # wait(100)
# # motorB.run_angle(-400,400,wait=True)
# # motorB.run_angle(-500,430,wait=False)
# # motorA.run_angle(-500,430,wait=True)
# # motorA.run_angle(-350,350,wait=True)
# # motorA.stop()
# # motorB.stop()
# # motorB.run(300)
# # motorA.run(300)
# # color = ColorSensor(Port.S1)
# # color2 = ColorSensor(Port.S2)
# # while True:
# #         if color.color() == Color.RED:
# #             motorA.stop()
# #             motorB.stop()
# #             break
# #         if color2.color() == Color.RED:
# #             motorA.stop()
# #             motorB.stop()
# #             break

# # motorA.run_angle(-300,50,wait=False)
# # motorB.run_angle(-300,50,wait=True)
# # motorA.stop()
# # motorB.stop()
# # drop.run_angle(300,-230,wait=True)
# # motorA.run_angle(-300,220,wait=False)
# # motorB.run_angle(-300,220,wait=True)
# # motorA.stop()
# # motorB.stop()
# # drop.run_angle(300,230,wait=True)
# # phase3()
# # phase4()

# # drop_house()

