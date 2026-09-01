#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
import Acceleration as acceleration
import MiscSetup as misc

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)
robot = DriveBase(motorA, motorB, wheel_diameter=62.4, axle_track=192)
print(ev3.battery.voltage())

acc = acceleration.AccelerationController(Kp=0.8)

if ev3.battery.voltage() >= 7000:
    # slider weewoo
    misc.reset_slider()
    motorC.run_target(-500, -1000) # (speed, angle)


    # Phase 1: Move Bucket and Scan Mosaic
    acc.line_following(890)
    acc.turn_degrees(90, mode="spot")
    acc.turn_degrees(-90, mode="spot")
    acc.line_following(740)
    acc.turn_degrees(-90, mode="spot")
    acc.line_following(460)
    acc.turn_degrees(-90, mode="spot")
    sensor_log = acc.move_colour_scan(600, default_max_speed=200)
    for i in sensor_log: print(i)

    




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