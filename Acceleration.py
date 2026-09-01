#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import Ev3devSensor
from LineTracer import PDController
import math

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)

wheel_rad = 31.2  
w2w_length = 192  # short for wheel to wheel length. im not writing allat
min_rad = 200   # arc radius for full steer (tune), usually more than half of wheel_to_wheel_length

def dist_to_angle(dist):
    return (dist / (math.pi * wheel_rad * 2)) * 360

def angle_to_dist(angle):
    return (angle / 360) * (math.pi * wheel_rad * 2)

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

    def move_distance(self, target_distance, default_min_speed = 50, default_max_speed=1200, default_ramp_dist=200):
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
            correction = error * self.Kp

            motorA.run(ideal_speed - correction)
            motorB.run(ideal_speed + correction)
            wait(10)

        motorA.stop()
        motorB.stop()

    def line_following(self, target_distance, default_min_speed = 50, default_max_speed=1200, default_ramp_dist=200, target_light=182, sensor=None, kp=0.2, kd=0.02):
        """
            Very similar to forward movement code, but it does so by following a line.
            The only difference is where it calculates error and subsequent correction from.
        """
        ramp_dist, cruise_dist, max_speed = self.dist_planning(target_distance, default_max_speed, default_ramp_dist)
        motorA.reset_angle(0)
        motorB.reset_angle(0)
        avg_dist, ideal_speed = 0, 0

        if sensor is None:
            sensor = Ev3devSensor(Port.S1)
        color_sensor = sensor # Pass in Ev3devSensor object
        pd_controller = PDController(kp=kp, kd=kd)

        while avg_dist < target_distance:
            avg_dist = (angle_to_dist(motorA.angle()) + angle_to_dist(motorB.angle())) / 2
            phase, progress = self.get_phase(avg_dist, ramp_dist, cruise_dist, target_distance)

            # PD Integration Code
            current_light = color_sensor.read('RGB')[-1]
            correction = pd_controller.calculate(target_light, current_light)
            
            if phase == "CRUISE":
                ideal_speed = max_speed
            else: # RAMP_UP or RAMP_DOWN
                ideal_speed = self.compute_ramp_speed(progress, default_min_speed, max_speed)


            motorA.run(ideal_speed - correction)
            motorB.run(ideal_speed + correction)
            wait(10)

        motorA.stop()
        motorB.stop()

    def turn_degrees(self, turn_angle, mode="spot", default_min_speed = 30, default_max_speed=500, default_ramp_dist=100):
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
                correction = error * self.Kp
    
                motorA.run(-orientation*(ideal_speed - correction))
                motorB.run(orientation*(ideal_speed + correction))
                wait(10)
            motorA.stop()
            motorB.stop()

        else: # mode == "arc"
            s = turn_angle / 90.0
            Rturn = max(min_rad, min_rad / abs(s))  # bigger radius for smaller angle
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
                correction = self.Kp * error

                outer_motor.run(outer_speed)
                inner_motor.run(inner_speed + correction)
                wait(10)
            motorA.stop()
            motorB.stop()