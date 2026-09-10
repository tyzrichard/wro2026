from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button
from Hardware import motorA, motorB, motorC, motorD, robot, leftColor, middleColor, rightColor, pushButton
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from LineTracer import PDController
import math

ev3 = EV3Brick()

wheel_rad = 31.2  # 63.76 wheel diameter as measured(?)
w2w_length = 191.28  # short for wheel to wheel length. im not writing allat
min_rad = 200   # arc radius for full steer (tune), usually more than half of wheel_to_wheel_length
beep_time = 0

def dist_to_angle(dist):
    return (dist / (math.pi * wheel_rad * 2)) * 360

def angle_to_dist(angle):
    return (angle / 360) * (math.pi * wheel_rad * 2)

def checkColor(r, g, b, color, buffer=25):
    if abs(color[0] - r) < buffer and abs(color[1] - g) < buffer and abs(color[2] - b) < buffer:
        return True
    return False

def clamp(value, lo, hi):
    return max(lo, min(hi, value))

def dist_planning(target_distance, default_max_speed, default_ramp_dist, decel_scale=1):
    """
        Returns the distances needed for the ramps (both are same), cruise and max speed achievable.
        If the distance is too short cruising is removed and the ramps scaled down proportionally.
        All input/outputs are in mm.
    """
    decel_dist = default_ramp_dist * decel_scale
    if target_distance >= default_ramp_dist + decel_dist:
        cruise_dist = target_distance - default_ramp_dist - decel_dist
        return default_ramp_dist, decel_dist, cruise_dist, default_max_speed
    else:
        scale = target_distance / (default_ramp_dist + decel_dist)
        ramp_dist = default_ramp_dist * scale
        decel_dist = decel_dist * scale
        cruise_dist = 0
        max_speed = default_max_speed * scale
        return ramp_dist, decel_dist, cruise_dist, max_speed
 
def get_phase(dist_travelled, ramp_dist, decel_dist, cruise_dist, total_dist):
    """
        Returns the appropriate phase (Ramp up, Cruise, Ramp down) and the progress for each ramp.
    """
    if dist_travelled < ramp_dist:
        return "RAMP_UP", (dist_travelled / ramp_dist)
    elif dist_travelled < ramp_dist + cruise_dist:
        return "CRUISE", 1
    else:
        remaining = total_dist - dist_travelled
        return "RAMP_DOWN", (remaining / decel_dist)
 
def compute_ramp_speed(progress, min_speed, max_speed):
    """
        Sigmoid-shaped speed target based on profile progress (unchanged from your original).
    """
    x = progress * 5 - 2.5
    sigmoid = 1 / (1 + math.exp(-x))
    return (max_speed - min_speed) * sigmoid + min_speed
 
def move_distance(target_distance, default_min_speed=20, default_max_speed=1000, default_ramp_dist=100):
    """
    Moves the robot forwards using direct duty-cycle (dc) control instead of motor.run().

    Architecture:
    1. Outer loop (same as before): sigmoid motion profile over distance travelled produces a target speed in deg/s.
    2. Inner loop (new): a PD+feedforward controller turns (target_speed - motor.speed()) into a duty command sent via motor.dc().
       This is the part that replaces Pybricks' internal run() PID with your own,faster, robot-specific one.
    3. Sync loop (upgraded from P-only to PD): keeps motorA/motorB travelling the same distance by trimming duty based on their distance error and its derivative.
    """
    kff = 0.12         # %/(deg/s)   -- TUNE FIRST, on a full battery
    kp_speed = 0.05     # %/(deg/s) of speed error
    kd_speed = 0.004    # %/(deg/s^2) of speed-error rate of change
    kp_sync = 0.01
    kd_sync = 0.002
    deriv_alpha = 0.5
    
    ramp_dist, decel_dist, cruise_dist, max_speed = dist_planning(abs(target_distance), default_max_speed, default_ramp_dist)
 
    motorA.reset_angle(0)
    motorB.reset_angle(0)
 
    direction = 1 if target_distance > 0 else -1
    target_distance = abs(target_distance)
 
    timer = StopWatch()
    prev_time = timer.time()
 
    prev_err_A, prev_err_B = 0, 0
    filt_deriv_A, filt_deriv_B = 0, 0
    prev_sync_error = 0
    filt_sync_deriv = 0
 
    avg_dist = 0
    print("moving")
    while avg_dist < target_distance:
        now = timer.time()
        dt = max(now - prev_time, 1)  # ms; guard against divide-by-zero on a fast tick
        prev_time = now
 
        # Distances travelled, normalised so "forward" is always positive regardless of direction.
        dist_A = angle_to_dist(direction * motorA.angle())
        dist_B = angle_to_dist(direction * motorB.angle())
        avg_dist = (dist_A + dist_B) / 2
 
        phase, progress = get_phase(avg_dist, ramp_dist, decel_dist, cruise_dist, target_distance)
        if phase == "CRUISE":
            target_speed = max_speed
        else:
            target_speed = compute_ramp_speed(progress, default_min_speed, max_speed)
 
        # ---- per-motor speed PD + feedforward -> duty ----
        speed_A = direction * motorA.speed()
        speed_B = direction * motorB.speed()
 
        err_A = target_speed - speed_A
        err_B = target_speed - speed_B
 
        deriv_A = (err_A - prev_err_A) / dt * 1000  # deg/s^2
        deriv_B = (err_B - prev_err_B) / dt * 1000
        filt_deriv_A = deriv_alpha * deriv_A + (1 - deriv_alpha) * filt_deriv_A
        filt_deriv_B = deriv_alpha * deriv_B + (1 - deriv_alpha) * filt_deriv_B
        prev_err_A, prev_err_B = err_A, err_B
 
        duty_A = kff * target_speed + kp_speed * err_A + kd_speed * filt_deriv_A
        duty_B = kff * target_speed + kp_speed * err_B + kd_speed * filt_deriv_B
 
        # ---- wheel sync PD (was P-only) ----
        sync_error = dist_A - dist_B
        sync_deriv = (sync_error - prev_sync_error) / dt * 1000
        filt_sync_deriv = deriv_alpha * sync_deriv + (1 - deriv_alpha) * filt_sync_deriv
        prev_sync_error = sync_error
 
        sync_correction = kp_sync * sync_error + kd_sync * filt_sync_deriv
 
        duty_A -= sync_correction
        duty_B += sync_correction
        print(duty_A, duty_B)

        motorA.dc(direction * clamp(duty_A, -100, 100))
        motorB.dc(direction * clamp(duty_B, -100, 100))
 
        wait(5)
 
    # brake() rather than stop(): with dc()-style control you generally want a firm,
    # repeatable stop rather than a coast that depends on residual momentum/friction.
    motorA.brake()
    motorB.brake()
    ev3.speaker.beep()
    wait(beep_time)