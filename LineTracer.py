#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
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