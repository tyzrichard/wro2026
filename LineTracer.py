from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

# Initialize EV3 Brick
ev3 = EV3Brick()

class PDController:
    def __init__(self, kp=0.0, kd=0.0, filter_alpha=0.9):
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
        self.previous_filtered_error = 0
        self.filter_alpha = filter_alpha  # 0 = no filtering, 1 = fully smoothed
        self.timer = StopWatch()
        self.timer.reset()
    
    def calculate(self, threshold_value, current_value):
        if self.filtered_value is None:
            self.filtered_value = current_value
        else:
            self.filtered_value = (self.filter_alpha * current_value + (1 - self.filter_alpha) * self.filtered_value)

        current_time = self.timer.time()
        error = threshold_value - current_value
        filtered_error = threshold_value - self.filtered_value

        p_term = self.kp * error * abs(error) / 100

        if current_time > 0:
            dt = current_time / 1000.0
            raw_derivative = (filtered_error - self.previous_filtered_error) / dt
            d_term = self.kd * raw_derivative
        else:
            raw_derivative = 0
            d_term = 0

        output = p_term + d_term

        self.last_derivative = raw_derivative   # <-- store here, before returning
        self.previous_error = error
        self.previous_filtered_error = filtered_error
        self.timer.reset()

        return output