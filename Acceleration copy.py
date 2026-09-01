#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.iodevices import Ev3devSensor
import math

# Import PD Controller from your line following file
from PDlinetracing import PDController

# Initialize EV3 Brick
ev3 = EV3Brick()

class AccelerationController:
    """
    Acceleration-Deceleration Controller using quadratic curves
    Provides smooth speed ramping like SPIKE Prime ACC block
    """
    def __init__(self, min_speed=0, max_speed=100, accel_time=0.5, decel_time=0.4):
        self.min_speed = min_speed  # mm/s
        self.max_speed = max_speed  # mm/s
        self.accel_time = accel_time * 1000  # convert to ms
        self.decel_time = decel_time * 1000  # convert ms
        self.speed_range = max_speed - min_speed  # Difference of speed in mm/s
        self.timer = StopWatch()
        self.target_distance = 0
        self.phase = "stopped"  # "accel", "cruise", "decel", "stopped"
        self._decel_distance = 0  # Cache deceleration distance

    def plan_for_distance(self, target_distance, desired_accel_time=None, desired_decel_time=None):
        T_a0 = (desired_accel_time or self.accel_time / 1000.0) # Accel time in s
        T_d0 = (desired_decel_time or self.decel_time / 1000.0) # Decel time in s

        def ramp_distance(T):
            return self.min_speed * T + (self.speed_range * T) / 3.0 

        D_a0 = ramp_distance(T_a0)
        D_d0 = ramp_distance(T_d0)
        ramp_total = D_a0 + D_d0 # dittances in mm

        if ramp_total <= target_distance:
            # Full ramp fits - use desired times, cruise fills the remainder
            self.accel_time = T_a0 * 1000.0
            self.decel_time = T_d0 * 1000.0
        else:
            # Not enough room - scale both ramps down proportionally (triangular profile)
            scale = target_distance / ramp_total if ramp_total > 0 else 0
            self.accel_time = (T_a0 * scale) * 1000.0
            self.decel_time = (T_d0 * scale) * 1000.0

        self.speed_range = self.max_speed - self.min_speed 
            
    def start(self, target_distance=None):
        """Start the acceleration profile"""
        self.timer.reset()
        self.target_distance = target_distance
        self.phase = "accel"
        # Pre-calculate deceleration distance when starting
        if target_distance:
            self._decel_distance = self._calculate_decel_distance()
        
    def _calculate_decel_distance(self):
        """a
        Calculate the distance needed to decelerate from max_speed to min_speed
        Using: d = (v^2 - u^2) / (2 * a)
        """
        if self.decel_time == 0:
            return 0
        decel_rate = self.speed_range / (self.decel_time * 0.001)  # More efficient conversion
        if decel_rate == 0:
            return 0
        return (self.max_speed**2 - self.min_speed**2) / (2 * decel_rate)
        
    def calculate_speed(self, current_position=None):
        """
        Calculate current speed using quadratic acceleration profile
        Returns speed in deg/s
        """
        if self.phase == "stopped":
            return 0
            
        elapsed_time = self.timer.time()
        
        if self.phase == "accel":
            if elapsed_time < self.accel_time:
                # Quadratic acceleration curve - optimized calculation
                progress = elapsed_time / self.accel_time
                speed_factor = progress * progress  # Quadratic curve
                return self.min_speed + self.speed_range * speed_factor
            else:
                self.phase = "cruise"
                return self.max_speed
                
        elif self.phase == "cruise":
            # Check if we need to start decelerating (only if target distance is set)
            if self.target_distance and current_position is not None:
                remaining_distance = self.target_distance - current_position
                
                if remaining_distance <= self._decel_distance:
                    self.phase = "decel"
                    self.timer.reset()
            return self.max_speed
            
        elif self.phase == "decel":
            if elapsed_time < self.decel_time:
                # Quadratic deceleration curve - optimized calculation
                progress = elapsed_time / self.decel_time
                speed_factor = (1 - progress) * (1 - progress)  # Inverse quadratic
                return self.min_speed + self.speed_range * speed_factor
            else:
                self.phase = "stopped"
                return 0
                
        return 0

class SynchronousController:
    """
    Synchronous Movement Controller
    Keeps two motors perfectly synchronized using encoder feedback
    """
    def __init__(self, kp=2.0):
        self.kp = kp
        self.slave_offset = 0
        
    def reset(self, master_motor, slave_motor):
        """Reset synchronization - call this before starting movement"""
        master_position = master_motor.angle()
        self.slave_offset = slave_motor.angle() - master_position
        
    def calculate_sync_correction(self, master_motor, slave_motor):
        """
        Calculate speed correction to keep motors synchronized
        Returns correction value to add to slave motor speed
        """
        current_master = master_motor.angle()
        current_slave = slave_motor.angle()
        
        # Expected slave position and synchronization error in one step
        sync_error = (current_master + self.slave_offset) - current_slave
        
        return self.kp * sync_error

def line_following(target_distance=500.0, target_light=182, sensor=Port.S1):
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = Ev3devSensor(sensor)

    robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=192)
    robot.reset()  # zero distance()/angle() so tracking starts fresh for this call

    pd_controller = PDController(kp=0.2, kd=0.02)
    acc_controller = AccelerationController(min_speed=100, max_speed=400)
    acc_controller.plan_for_distance(target_distance, desired_accel_time=0.5, desired_decel_time=0.4)
    acc_controller.start(target_distance=target_distance)

    try:
        while acc_controller.phase != "stopped":
            # 1. Read current light
            current_light = color_sensor.read('RGB')[-1]

            # 2. Compute turn rate using working PD logic
            turn_rate = pd_controller.calculate(target_light, current_light)

            # 3. Encoder feedback: how far the robot has actually traveled (mm)
            current_position = abs(robot.distance())

            # 4. Feed real position into the speed controller so it can decelerate on time
            current_speed = acc_controller.calculate_speed(current_position)

            # 5. Drive
            robot.drive(current_speed, turn_rate)

            wait(5)

    finally:
        robot.stop()

def line_following_blackvar(target_distance=100.0, target_light=182):
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    left_sensor = Ev3devSensor(Port.S1)
    color_sensor = Ev3devSensor(Port.S2)
    right_sensor = Ev3devSensor(Port.S3)
    
    robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=192)
    robot.reset()  # zero distance()/angle() so tracking starts fresh for this call
    
    pd_controller = PDController(kp=0.15, kd=0.02)
    acc_controller = AccelerationController(min_speed=100, max_speed=400)
    acc_controller.plan_for_distance(target_distance, desired_accel_time=0.5, desired_decel_time=0.4)
    acc_controller.start(target_distance=target_distance)
    
    try:
        while acc_controller.phase != "stopped":
            # 1. Read current light. Stop if black line has been reached.
            current_light = color_sensor.read('RGB')[-1]
            if (left_sensor.read('RGB')[-1] < target_light) and (color_sensor.read('RGB')[-1] < target_light) and (right_sensor.read('RGB')[-1] < target_light):
                break

            # 2. Compute turn rate using working PD logic
            turn_rate = pd_controller.calculate(target_light, current_light)
    
            # 3. Encoder feedback: how far the robot has actually traveled (mm)
            current_position = abs(robot.distance())
    
            # 4. Feed real position into the speed controller so it can decelerate on time
            current_speed = acc_controller.calculate_speed(current_position)
    
            # 5. Drive
            if target_distance >= 100:
                robot.drive(current_speed, turn_rate)
            else:
                robot.drive(1000, turn_rate)
    
            wait(5)
    
    finally:
        robot.stop()


# def acceleration_pd_line_following_drivebase(target_distance=5.0, target_light=182):
#     target_angle = (target_distance / (math.pi * 62.4)) * 360

#     left_motor = Motor(Port.A)
#     right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
#     color_sensor = Ev3devSensor(Port.S1)
    
#     robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=192)
    
#     pd_controller = PDController(kp=0.06, kd=0.02)
#     acc_controller = AccelerationController(min_speed=150, max_speed=800)
#     acc_controller.plan_for_distance(target_angle, desired_accel_time=0.5, desired_decel_time=0.4)
#     stopwatch = StopWatch()
    
#     acc_controller.start(target_distance=target_angle)
#     stopwatch.reset()
    
#     target_ms = duration_sec * 1000.0
    
#     try:
#         while stopwatch.time() < target_ms:
#             # 1. Read current light
#             current_light = color_sensor.read('RGB')[-1]
            
#             # 2. Compute turn rate using working PD logic
#             turn_rate = pd_controller.calculate(target_light, current_light)
            
#             # 3. Get smooth dynamic linear speed (mm/s)
#             current_speed = acc_controller.calculate_speed()
            
#             # 4. Feed both directly into DriveBase!
#             robot.drive(current_speed, turn_rate)
            
#             wait(5)
            
#     finally:
#         robot.stop()
#         ev3.screen.clear()
#         ev3.screen.print("Done!")

def synchronous_drive(target_distance, left_motor = Motor(Port.A), right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)):
    """
    Example 5: Acceleration-Deceleration + Synchronous Controller
    Smooth acceleration with perfect motor synchronization for straight driving
    """
    target_angle = (target_distance / (math.pi * 62.4)) * 360
    
    # Controllers and constants
    acc_controller = AccelerationController(min_speed=150, max_speed=800)
    acc_controller.plan_for_distance(target_angle, desired_accel_time=0.5, desired_decel_time=0.4)
    sync_controller = SynchronousController(kp=1)
    
    # Reset and start
    sync_controller.reset(left_motor, right_motor)
    acc_controller.start(target_distance=target_angle)
    
    start_position = left_motor.angle()
    
    try:
        while True:
            current_position = abs(left_motor.angle() - start_position)
            
            # Get smooth base speed with deceleration consideration (deg/s)
            base_speed = acc_controller.calculate_speed(current_position)
            
            if base_speed <= 5:  # Near stop
                break
                
            # Perfect synchronization
            sync_correction = sync_controller.calculate_sync_correction(left_motor, right_motor)
            
            # Apply speeds
            left_motor.run(base_speed)
            right_motor.run(base_speed + sync_correction)
            
            # Display
            # ev3.screen.clear()
            # ev3.screen.print("Speed:", int(base_speed), "deg/s")
            # ev3.screen.print("Distance:", current_position)
            # ev3.screen.print("Target:", target_distance)
            # ev3.screen.print("Phase:", acc_controller.phase)
            
            wait(25)
            
    except:
        pass
    
    left_motor.stop()
    right_motor.stop()
    # ev3.screen.clear()
    # ev3.screen.print("Speed:", int(base_speed), "deg/s")
    # ev3.screen.print("Distance:", current_position)
    # ev3.screen.print("Target:", target_distance)
    # ev3.screen.print("Phase:", acc_controller.phase)


def acceleration_pd_line_following_phase1(time = -1,speed = 600):
    """
    Example 4: Acceleration-Deceleration Controller + PD Controller
    Smooth acceleration with responsive steering for line following
    """
    # Initialize motors and sensor
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = ColorSensor(Port.S1)
    left_color = ColorSensor(Port.S2)
    
    # Controllers
    acc_controller = AccelerationController(min_speed=80, max_speed=speed, accel_time=0.5, decel_time=0.4)
    pd_controller = PDController(kp=1.2, kd=0.4)
    stopwatch = StopWatch()
    # Constants
    target_light = 30
    max_steering = 30
    max_motor_speed = 500
    
    acc_controller.start()
    stopwatch.reset()
    try:
        while True:
    # If time limit is set
            if time >= 0 and stopwatch.time() >= time * 1000:
                # Time is up → check reflections
                if left_color.reflection() <= 15:
                    break   # reflections are in range, so stop
                # else keep running

            # Get smooth base speed from acceleration controller (deg/s)
            base_speed = acc_controller.calculate_speed()
            
            # Get steering correction from PD controller
            current_light = color_sensor.reflection()
            steering = pd_controller.calculate(target_light, current_light)
            
            # Clamp steering to prevent huge speed differences
            if steering > max_steering:
                steering = max_steering
            elif steering < -max_steering:
                steering = -max_steering
            
            # Apply steering
            left_speed = base_speed - steering
            right_speed = base_speed + steering
            
            # Proportional scaling if speeds exceed limit
            max_speed = max(abs(left_speed), abs(right_speed))
            if max_speed > max_motor_speed:
                scale_factor = max_motor_speed / max_speed
                left_speed *= scale_factor
                right_speed *= scale_factor
            
            # Run motors
            left_motor.run(left_speed)
            right_motor.run(right_speed)
            
            # Display (reduced string operations)
            ev3.screen.clear()
            ev3.screen.print("Base Speed:", int(base_speed))
            ev3.screen.print("Light:", current_light)
            ev3.screen.print("Steering:", int(steering))
            ev3.screen.print("L:", int(left_speed), "R:", int(right_speed))
            
            wait(20)
            
    except:
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")
    finally:
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")
def acceleration_pd_line_following_phase3(time = -1,speed = 600):
    """
    Example 4: Acceleration-Deceleration Controller + PD Controller
    Smooth acceleration with responsive steering for line following
    """
    # Initialize motors and sensor
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = ColorSensor(Port.S1)
    left_color = ColorSensor(Port.S2)
    
    # Controllers
    acc_controller = AccelerationController(min_speed=80, max_speed=speed, accel_time=0.5, decel_time=0.4)
    pd_controller = PDController(kp=1.2, kd=0.4)
    stopwatch = StopWatch()
    # Constants
    target_light = 30
    max_steering = 30
    max_motor_speed = 500
    
    acc_controller.start()
    stopwatch.reset()
    try:
        while True:
    # If time limit is set
            if time >= 0 and stopwatch.time() >= time * 1000:
                # Time is up → check reflections
                if left_color.reflection() <=13:
                    break   # reflections are in range, so stop
                # else keep running

            # Get smooth base speed from acceleration controller (deg/s)
            base_speed = acc_controller.calculate_speed()
            
            # Get steering correction from PD controller
            current_light = color_sensor.reflection()
            steering = pd_controller.calculate(target_light, current_light)
            
            # Clamp steering to prevent huge speed differences
            if steering > max_steering:
                steering = max_steering
            elif steering < -max_steering:
                steering = -max_steering
            
            # Apply steering
            left_speed = base_speed - steering
            right_speed = base_speed + steering
            
            # Proportional scaling if speeds exceed limit
            max_speed = max(abs(left_speed), abs(right_speed))
            if max_speed > max_motor_speed:
                scale_factor = max_motor_speed / max_speed
                left_speed *= scale_factor
                right_speed *= scale_factor
            
            # Run motors
            left_motor.run(left_speed)
            right_motor.run(right_speed)
            
            # Display (reduced string operations)
            ev3.screen.clear()
            ev3.screen.print("Base Speed:", int(base_speed))
            ev3.screen.print("Light:", current_light)
            ev3.screen.print("Steering:", int(steering))
            ev3.screen.print("L:", int(left_speed), "R:", int(right_speed))
            
            wait(20)
            
    except:
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")
    finally:
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")
def pd_line_following_backwards(time=-1, speed=200):
    """
    PD Line Following (Backwards) for a fixed time
    - Runs backwards at given speed
    - Uses PD controller for steering
    """

    # Initialize motors and sensors
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = ColorSensor(Port.S2)   # Line follower
    color_sensor_right = ColorSensor(Port.S1)
    stopwatch = StopWatch()

    # PD controller
    pd_controller = PDController(kp=1.6, kd=0.4)

    # Constants
    target_light = 40
    max_steering = 50
    max_motor_speed = abs(speed)  # always positive
    backwards_speed = abs(speed)  # ensure negative for backwards

    stopwatch.reset()
    try:
        while True:
            if(stopwatch.time() > time * 1000):
                if color_sensor_right.reflection() <40:
                    break
            # Steering correction (inverted for backwards)
            current_light = color_sensor.reflection()
            steering = -pd_controller.calculate(target_light, current_light)

            # Clamp steering
            steering = max(min(steering, max_steering), -max_steering)

            # Apply steering
            left_speed = backwards_speed + steering
            right_speed = backwards_speed - steering

            # Scale if necessary
            max_speed_val = max(abs(left_speed), abs(right_speed))
            if max_speed_val > max_motor_speed:
                scale_factor = max_motor_speed / max_speed_val
                left_speed *= scale_factor
                right_speed *= scale_factor

            # Run motors
            left_motor.run(left_speed)
            right_motor.run(right_speed)

            # Debug screen
            ev3.screen.clear()
            ev3.screen.print("Backwards Line Follow")
            ev3.screen.print("Light:", current_light)
            ev3.screen.print("Steer:", int(steering))
            ev3.screen.print("L:", int(left_speed), "R:", int(right_speed))

            wait(20)

    finally:
        # Always stop motors at the end
        wait(100)
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")

def synchronous_movement():
    """
    Example 3: Synchronous Movement Controller
    Both motors move together with perfect synchronization
    """
    # Initialize motors
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    
    # Controllers
    acc_controller = AccelerationController(min_speed=120, max_speed=360, accel_time=3.0, decel_time=2.0)
    sync_controller = SynchronousController(kp=1.5)
    
    # Constants
    target_distance = 1080
        
    # Reset synchronization
    sync_controller.reset(left_motor, right_motor)
    acc_controller.start(target_distance=target_distance)
    
    try:
        while True:
            # Get smooth base speed (deg/s)
            base_speed = acc_controller.calculate_speed(left_motor.angle())
            
            if base_speed <= 0:
                break  # Movement complete
            
            # Calculate synchronization correction
            sync_correction = sync_controller.calculate_sync_correction(left_motor, right_motor)
            
            # Apply synchronization (right motor follows left motor)
            left_motor.run(base_speed)
            right_motor.run(base_speed + sync_correction)
            
            # Display
            ev3.screen.clear()
            ev3.screen.print("Speed:", int(base_speed), "deg/s")
            ev3.screen.print("L pos:", left_motor.angle())
            ev3.screen.print("R pos:", right_motor.angle())
            ev3.screen.print("Sync corr:", int(sync_correction))
            
            wait(20)
            
    except:
        pass
    
    # Stop motors
    left_motor.stop()
    right_motor.stop()
    ev3.screen.clear()
    ev3.screen.print("Movement complete")
    wait(2000)


def main():
    """Main menu to choose controller example"""
    while True:
        ev3.screen.clear()
        ev3.screen.print("Advanced Controllers")
        ev3.screen.print("")
        ev3.screen.print("UP: Accel+PD Line")
        ev3.screen.print("DOWN: Synchronous")
        ev3.screen.print("LEFT: Accel+Sync Drive")
        ev3.screen.print("CENTER: Exit")
        
        pressed = ev3.buttons.pressed()
        
        if pressed:
            if Button.UP in pressed:
                acceleration_pd_line_following()
            elif Button.DOWN in pressed:
                synchronous_movement()
            elif Button.LEFT in pressed:
                acceleration_synchronous_drive()
            elif Button.CENTER in pressed:
                ev3.speaker.say("Goodbye")
                break
        
        wait(100)

if __name__ == "__main__":
    main()