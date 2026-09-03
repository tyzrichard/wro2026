#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import Ev3devSensor
from pybricks.robotics import DriveBase
from LineTracer import PDController
import math

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
leftColor = Ev3devSensor(Port.S1)
middleColor = Ev3devSensor(Port.S2)
rightColor = Ev3devSensor(Port.S3)
robot = DriveBase(motorA, motorB, wheel_diameter=62.4, axle_track=192)

wheel_rad = 31.2  
w2w_length = 192  # short for wheel to wheel length. im not writing allat
min_rad = 200   # arc radius for full steer (tune), usually more than half of wheel_to_wheel_length
beep_time = 0

def dist_to_angle(dist):
    return (dist / (math.pi * wheel_rad * 2)) * 360

def angle_to_dist(angle):
    return (angle / 360) * (math.pi * wheel_rad * 2)

def checkColor(r, g, b, color, buffer=40):
    if abs(color[0] - r) < buffer and abs(color[1] - g) < buffer and abs(color[2] - b) < buffer:
        return True
    return False

class AccelerationController:
    def __init__(self, Kp=0.5):
        self.Kp = Kp

    def dist_planning(self, target_distance, default_max_speed, default_ramp_dist):
        """
            Returns the distances needed for the ramps (both are same), cruise and max speed achievable.
            If the distance is too short cruising is removed and the ramps scaled down proportionally.
            All input/outputs are in mm.
        """
        if target_distance >= default_ramp_dist * 2: # Distance is sufficient
            cruise_dist = target_distance - (default_ramp_dist * 2)
            return default_ramp_dist, cruise_dist, default_max_speed
        else: # Distance is NOT sufficient
            ramp_dist = target_distance / 2
            cruise_dist = 0
            max_speed = default_max_speed * (ramp_dist / default_ramp_dist)
            # I got lazy and just calculated max_speed proportionally. Distance covered will still be the same.
            return ramp_dist, cruise_dist, max_speed

    def get_phase(self, dist_travelled, ramp_dist, cruise_dist, total_dist):
        """
            Returns the appropriate phase (Ramp up, Cruise, Ramp down) and the progress for each ramp for speed scaling.
            All input/outputs are in mm.
        """
        if dist_travelled < ramp_dist: # Phase 1: Speeding up
            return "RAMP_UP", (dist_travelled / ramp_dist) # From 0 to 1
        elif dist_travelled < ramp_dist + cruise_dist: # Phase 2: Cruising
            return "CRUISE", 1
        else:
            remaining = total_dist - dist_travelled # Phase 3: Slowing down
            return "RAMP_DOWN", (remaining / ramp_dist) # From 1 to 0
    
    def compute_ramp_speed(self, progress, min_speed, max_speed):
        """
            Returns the speed the robot should move at based on its current distance.
            The base sigmoid function goes from ~0 to ~1 from x=-6 to x=6, so we change progress to suit that.
            We decided x=-2 to x=2 worked better as the gradients at the extremes were too low.
        """
        x = progress * 5 - 2.5
        sigmoid = 1 / (1 + math.exp(-x))
        return (max_speed - min_speed) * sigmoid + min_speed

    def move_distance(self, target_distance, default_min_speed = 50, default_max_speed=2000, default_ramp_dist=200):
        """
            Moves the robot forwards with smooth acceleration and motor synchronisation.
            1. Calculates distances for all three phases
            2. Resets Encoders for both motors to 0
            3. For each loop, it gets Phase and Progress based on encoder distances, and calculates the speed required.
            4. Calculates Error from previous loop and factors it when running motors.
        """
        ramp_dist, cruise_dist, max_speed = self.dist_planning(target_distance, default_max_speed, default_ramp_dist)
        motorA.reset_angle(0)
        motorB.reset_angle(0)
        avg_dist, ideal_speed = 0, 0

        while avg_dist < target_distance:
            avg_dist = (angle_to_dist(motorA.angle()) + angle_to_dist(motorB.angle())) / 2
            phase, progress = self.get_phase(avg_dist, ramp_dist, cruise_dist, target_distance)
            
            if phase == "CRUISE":
                ideal_speed = max_speed
            else: # RAMP_UP or RAMP_DOWN
                ideal_speed = self.compute_ramp_speed(progress, default_min_speed, max_speed)

            error = angle_to_dist(motorA.angle()) - angle_to_dist(motorB.angle())
            print(error)
            correction = abs(error) * error * self.Kp 

            motorA.run(ideal_speed - correction)
            motorB.run(ideal_speed + correction)
            wait(5)

        motorA.stop()
        motorB.stop()
        ev3.speaker.beep()
        wait(beep_time)

    def move_colour_scan(self, target_distance, last_sensor_dist=200, scan_interval_dist=50, default_min_speed=50, default_max_speed=200, default_ramp_dist=100):
        """
            Forward moving code with extra bits added to log colours starting from the mosaic's black border and every scan_interval_dist afterwards.
        """
        ramp_dist, cruise_dist, max_speed = self.dist_planning(target_distance, default_max_speed, default_ramp_dist)
        motorA.reset_angle(0)
        motorB.reset_angle(0)
        avg_dist, ideal_speed = 0, 0
        last_sensor_dist -= scan_interval_dist

        sensor_log = []

        while avg_dist < target_distance:
            avg_dist = (angle_to_dist(motorA.angle()) + angle_to_dist(motorB.angle())) / 2
            phase, progress = self.get_phase(avg_dist, ramp_dist, cruise_dist, target_distance)
            
            if phase == "CRUISE":
                ideal_speed = max_speed
            else: 
                ideal_speed = self.compute_ramp_speed(progress, default_min_speed, max_speed)

            error = angle_to_dist(motorA.angle()) - angle_to_dist(motorB.angle())
            correction = abs(error) * error * self.Kp / 100

            motorA.run(ideal_speed - correction)
            motorB.run(ideal_speed + correction)

            if  avg_dist - last_sensor_dist >= scan_interval_dist and len(sensor_log) < 4:
                sensor_line = []
                colorReads = [leftColor.read('RGB'), middleColor.read('RGB'), rightColor.read('RGB')]
                for color in colorReads:
                    if checkColor(115, 116, 120, color):
                        sensor_line.append("White")
                    elif checkColor(110, 93, 35, color):
                        sensor_line.append("Yellow")
                    elif checkColor(15, 30, 65, color):
                        sensor_line.append("Blue")
                    elif checkColor(25, 38, 30, color):
                        sensor_line.append("Green")
                    else:
                        sensor_line.append("NA" + str(color))
                    # sensor_line.append(color) # for calibration testing
                sensor_log.append(sensor_line)
                last_sensor_dist = avg_dist
            wait(10)

        motorA.stop()
        motorB.stop()
        ev3.speaker.beep()
        wait(beep_time)
        return sensor_log

    def line_following(self, target_distance, default_min_speed=50, default_max_speed=600, default_ramp_dist=200, target_light=162, sensor=None, kp=0.1, kd=0.0000):
        """
            Very similar to forward movement code, but it does so by following a line.
            The only difference is where it calculates error and subsequent correction from.
        """
        ramp_dist, cruise_dist, max_speed = self.dist_planning(target_distance, default_max_speed, default_ramp_dist)
        robot.reset() 
        avg_dist, ideal_speed = 0, 0

        if sensor is None:
            sensor = Ev3devSensor(Port.S1)
        color_sensor = sensor # Pass in Ev3devSensor object
        pd_controller = PDController(kp=kp, kd=kd)

        while avg_dist < target_distance:
            avg_dist = robot.distance()
            phase, progress = self.get_phase(avg_dist, ramp_dist, cruise_dist, target_distance)

            # PD Integration Code
            current_light = color_sensor.read('RGB')[-1]
            turn_rate = pd_controller.calculate(target_light, current_light) 
            
            if phase == "CRUISE":
                ideal_speed = max_speed
            else: # RAMP_UP or RAMP_DOWN
                ideal_speed = self.compute_ramp_speed(progress, default_min_speed, max_speed)

            # Clamp turn_rate so it can't overwhelm ideal_speed at low speeds
            max_turn_rate = ideal_speed * 0.8  # tune this multiplier
            turn_rate = max(-max_turn_rate, min(max_turn_rate, turn_rate))

            robot.drive(ideal_speed, turn_rate)
            wait(5)

        robot.stop()
        ev3.speaker.beep()
        wait(beep_time)

    def blackstop(self, creep_speed=50, left_target_light=125, right_target_light=135, buffer=20, filter_alpha=0.75, kp=0.5):
        """
            Moves the vehicle slowly towards a black line and stops. 
            Left and right sensors are used to reposition the wheels.
        """
        motorA.reset_angle(0)
        motorB.reset_angle(0)
        creep_angle = dist_to_angle(creep_speed)

        filtered_left = None
        filtered_right = None

        left_light = leftColor.read('RGB')[-1]
        right_light = rightColor.read('RGB')[-1]

        stable = 0

        while stable <= 5:
            raw_left = leftColor.read('RGB')[-1]
            raw_right = rightColor.read('RGB')[-1]

            if filtered_left is None:
                filtered_left = raw_left
                filtered_right = raw_right
            else:
                filtered_left = filter_alpha * raw_left + (1 - filter_alpha) * filtered_left
                filtered_right = filter_alpha * raw_right + (1 - filter_alpha) * filtered_right

            left_light = filtered_left
            right_light = filtered_right

            left_good = abs(left_light - left_target_light) <= buffer
            right_good = abs(right_light - right_target_light) <= buffer

            MIN_FORWARD_SPEED = 12
            MIN_REVERSE_SPEED = 6

            left_error = left_light - left_target_light
            if abs(left_error) <= buffer:
                motorB.hold()
            else:
                left_speed = kp * left_error
                left_speed = max(-creep_angle, min(creep_angle, left_speed))

                if 0 < left_speed < MIN_FORWARD_SPEED:
                    left_speed = MIN_FORWARD_SPEED
                elif -MIN_REVERSE_SPEED < left_speed < 0:
                    left_speed = -MIN_REVERSE_SPEED

                motorB.run(left_speed)

            right_error = right_light - right_target_light
            if abs(right_error) <= buffer:
                motorA.hold()
            else:
                right_speed = kp * right_error

                right_speed = max(-creep_angle, min(creep_angle, right_speed))

                if 0 < right_speed < MIN_FORWARD_SPEED:
                    right_speed = MIN_FORWARD_SPEED
                elif -MIN_REVERSE_SPEED < right_speed < 0:
                    right_speed = -MIN_REVERSE_SPEED

                motorA.run(right_speed)

            if left_good and right_good:
                stable += 1
            else:
                stable = 0
            wait(5)

        # print(str(leftColor.read('RGB')[-1]) + "   " + str(middleColor.read('RGB')[-1]) + "  " + str(rightColor.read('RGB')[-1]))

    def line_following_blackvar(self, min_speed=50, max_speed=100, ramp_dist=100, target_light=162, black_buffer=60, sensor=None, kp=0.07, kd=0.007):
        """
            Very similar to forward movement code, but it does so by following a line.
            The only difference is where it calculates error and subsequent correction from.
        """
        robot.reset() 
        avg_dist, ideal_speed = 0, 0
    
        if sensor is None:
            sensor = Ev3devSensor(Port.S2)
        color_sensor = sensor # Pass in Ev3devSensor object
        pd_controller = PDController(kp=kp, kd=kd)
    
        while leftColor.read('RGB')[-1] >= (target_light + black_buffer) and rightColor.read('RGB')[-1] >= (target_light + black_buffer):
            avg_dist = robot.distance()
    
            # PD Integration Code
            current_light = color_sensor.read('RGB')[-1]
            turn_rate = pd_controller.calculate(target_light, current_light)
                
            if avg_dist >= ramp_dist:
                ideal_speed = max_speed
            else: # RAMP_UP. I just threw the sigmoid code here.
                progress = avg_dist / ramp_dist
                x = progress * 5 - 2.5
                sigmoid = 1 / (1 + math.exp(-x))
                ideal_speed = (max_speed - min_speed) * sigmoid + min_speed

            max_turn_rate = ideal_speed * 0.8   # tune this multiplier
            turn_rate = max(-max_turn_rate, min(max_turn_rate, turn_rate))
    
            robot.drive(ideal_speed, turn_rate)
            wait(5)
        robot.stop()
        self.blackstop()
        ev3.speaker.beep()
        wait(beep_time)

    def turn_degrees(self, turn_angle, mode="spot", turn_radius=min_rad, default_min_speed = 30, default_max_speed=700, default_ramp_dist=100):
        """
            Makes the robot perform spot (tank) steering and arc steering.
            Spot is very similar to the move_distance function where the only difference is the wheel direction.
            Arc plans the outer wheel ONLY and scaled the inner wheel accordingly. ONLY the inner wheel is corrected.
        """
        if mode == "spot":
            wheel_dist = angle_to_dist(abs(turn_angle) * (w2w_length / (2 * wheel_rad)))
            orientation = 1 if turn_angle > 0 else -1

            ramp_dist, cruise_dist, max_speed = self.dist_planning(wheel_dist, default_max_speed, default_ramp_dist)
            motorA.reset_angle(0)
            motorB.reset_angle(0)

            avg_dist, ideal_speed = 0, 0
    
            while avg_dist < wheel_dist:
                avg_dist = (abs(angle_to_dist(motorA.angle())) + abs(angle_to_dist(motorB.angle()))) / 2
                phase, progress = self.get_phase(avg_dist, ramp_dist, cruise_dist, wheel_dist)
                
                if phase == "CRUISE":
                    ideal_speed = max_speed
                else: # RAMP_UP or RAMP_DOWN
                    ideal_speed = self.compute_ramp_speed(progress, default_min_speed, max_speed)
    
                error = abs(angle_to_dist(motorA.angle())) - abs(angle_to_dist(motorB.angle()))
                correction = abs(error) * error * self.Kp / 100
    
                motorA.run(-orientation*(ideal_speed - correction))
                motorB.run(orientation*(ideal_speed + correction))
                wait(10)
            motorA.stop()
            motorB.stop()

        else: # mode == "arc"
            s = turn_angle / 90.0
            Rturn = max(turn_radius, turn_radius / abs(s))  # bigger radius for smaller angle
            center_arc_len = (abs(turn_angle) * math.pi/180) * Rturn

            outer_target = center_arc_len * (Rturn + w2w_length / 2) / Rturn
            inner_target = center_arc_len * (Rturn - w2w_length / 2) / Rturn
            if turn_angle > 0:
                # Right turn: left wheel (motorB) is outer, travels more
                outer_motor, inner_motor = motorB, motorA
            else:
                # Left turn: right wheel (motorA) is outer, travels more
                outer_motor, inner_motor = motorA, motorB

            ratio = inner_target / outer_target

            ramp_dist, cruise_dist, outer_max_speed = self.dist_planning(outer_target, default_max_speed, default_ramp_dist)

            motorA.reset_angle(0)
            motorB.reset_angle(0)
            outer_dist_travelled = 0

            while outer_dist_travelled < outer_target:
                outer_dist_travelled = angle_to_dist(outer_motor.angle())
                phase, progress = self.get_phase(outer_dist_travelled, ramp_dist, cruise_dist, outer_target)

                if phase == "CRUISE":
                    outer_speed = outer_max_speed
                else:
                    outer_speed = self.compute_ramp_speed(progress, default_min_speed, outer_max_speed)
                inner_speed = outer_speed * ratio

                inner_dist_travelled = angle_to_dist(inner_motor.angle())
                expected_inner_dist = outer_dist_travelled * ratio
                error = inner_dist_travelled - expected_inner_dist
                correction = error * abs(error) * self.Kp / 100

                outer_motor.run(outer_speed)
                inner_motor.run(inner_speed + correction)
                wait(10)
            motorA.stop()
            motorB.stop()
        ev3.speaker.beep()
        wait(beep_time)