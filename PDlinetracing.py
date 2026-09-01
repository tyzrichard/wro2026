#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.iodevices import Ev3devSensor

# Initialize EV3 Brick
ev3 = EV3Brick()

class PDController:
    def __init__(self, kp=0.1, kd=0.0, filter_alpha=0.2):
        """
        PD Controller implementation
        
        Args:
            kp (float): Proportional gain
            kd (float): Derivative gain
        """
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
        self.filtered_value = None
        self.filter_alpha = filter_alpha  # 0 = no filtering, 1 = fully smoothed
        self.timer = StopWatch()
        self.timer.reset()
    
    def calculate(self, threshold_value, current_value):
        """
        Calculate PD controller output
        
        Args:
            threshold_value (float): Desired value (threshold/target)
            current_value (float): Current measured value
            
        Returns:
            float: Controller output
        """
        if self.filtered_value is None:
            self.filtered_value = current_value
        else:
            self.filtered_value = (self.filter_alpha * current_value +
                                    (1 - self.filter_alpha) * self.filtered_value)

        current_time = self.timer.time()
        error = threshold_value - current_value
        p_term = self.kp * error
        
        # Derivative term
        if current_time > 0:
            dt = current_time / 1000.0  # Convert ms to seconds
            d_term = self.kd * (error - self.previous_error) / dt
        else:
            d_term = 0
        
        # PD output
        output = p_term + d_term
        
        # Update for next iteration
        self.previous_error = error
        self.timer.reset()
        
        return output

def line_following_basic(): #not used
    """
    Basic line following using PD controller with individual motor control
    """
    # Initialize motors and sensor
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = ColorSensor(Port.S1)
    
    # PD Controller setup
    pd_controller = PDController(kp=1.5, kd=0.4)
    
    # Target value (edge of line - adjust based on your setup)
    target_light = 30  # Adjust this value based on your line/surface
    
    # Base speed
    base_speed = 500  # degrees per second
    
    
    try:
        while True:
            # Read current light value
            current_light = color_sensor.reflection()
            
            # Calculate steering correction using PD controller
            steering = pd_controller.calculate(target_light, current_light)
            
            # Apply steering to motors
            left_speed = base_speed - steering
            right_speed = base_speed + steering
            
            # Constrain motor speeds
            left_speed = max(min(left_speed, 500), -500)
            right_speed = max(min(right_speed, 500), -500)
            
            # Set motor speeds
            left_motor.run(left_speed)
            right_motor.run(right_speed)
            
            # Display values on screen
            ev3.screen.clear()
            ev3.screen.print("Light:", current_light)
            ev3.screen.print("Steering:", int(steering))
            ev3.screen.print("L:", int(left_speed))
            ev3.screen.print("R:", int(right_speed))
            
            # Small delay
            wait(10)
    except:
        # Stop motors on exit
        left_motor.stop()
        right_motor.stop()
        ev3.screen.clear()
        ev3.screen.print("Stopped")


def line_following_drivebase():
    """
    Line following using DriveBase and PD controller (recommended approach)
    """
    # Initialize motors and sensor
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    color_sensor = Ev3devSensor(Port.S1) #left sensor
    
    # Create drive base (adjust wheel_diameter and axle_track for your robot)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=192)
    
    # PD Controller setup
    pd_controller = PDController(kp=0.1, kd=0.0005)
    
    # Target value
    threshold = 182
    
    # Drive speed (mm/s)
    drive_speed = 400
    
    
    try:
        while True:
            # Read sensor
            current_light = color_sensor.read('RGB')[-1]
            
            # Calculate turn rate
            turn_rate = pd_controller.calculate(threshold, current_light)
            
            # Use DriveBase drive method
            robot.drive(drive_speed, turn_rate*2)
            
            # Display values on screen
            ev3.screen.clear()
            ev3.screen.print("Light:", current_light)
            ev3.screen.print("Turn rate:", int(turn_rate))
            ev3.screen.print("Speed:", drive_speed, "mm/s")
            
            wait(10)
            
    except KeyboardInterrupt:
        robot.stop()

def calibrate_sensor():
    """
    Helper function to calibrate the color sensor
    Find the light values for your line and surface
    """
    color_sensor = Ev3devSensor(Port.S1)
    
    ev3.screen.clear()
    ev3.screen.print("Calibrating...")
    ev3.screen.print("Move sensor over")
    ev3.screen.print("line and surface")
    ev3.screen.print("Press CENTER to exit")
    
    min_val = 255
    max_val = 0
    
    try:
        while True:
            current_light = color_sensor.read('RGB')[-1]
            
            if current_light < min_val:
                min_val = current_light
            if current_light > max_val:
                max_val = current_light
            
            ev3.screen.clear()
            ev3.screen.print("Current:", current_light)
            ev3.screen.print("Min (dark):", min_val)
            ev3.screen.print("Max (light):", max_val)
            ev3.screen.print("Target:", (min_val + max_val) // 2)
            ev3.screen.print("CENTER to exit")
            
            if Button.CENTER in ev3.buttons.pressed():
                break
                
            wait(100)
            
    except KeyboardInterrupt:
        pass
    
    target_value = (min_val + max_val) // 2
    ev3.screen.clear()
    ev3.screen.print("Calibration Results:")
    ev3.screen.print("Dark value:", min_val)
    ev3.screen.print("Light value:", max_val) 
    ev3.screen.print("Suggested:", target_value)
    wait(3000)

