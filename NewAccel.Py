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
        scale = target_distance / (default_ramp_dist + decel_dist) if (default_ramp_dist + decel_dist) else 0
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
        return "RAMP_UP", (dist_travelled / ramp_dist) if ramp_dist else 1
    elif dist_travelled < ramp_dist + cruise_dist: 
        return "CRUISE", 1
    else:
        remaining = total_dist - dist_travelled
        return "RAMP_DOWN", (remaining / decel_dist) if decel_dist else 0
 
def compute_ramp_speed(progress, min_speed, max_speed):
    """
    Sigmoid ramp normalized so:
        progress = 0 -> exactly min_speed
        progress = 1 -> exactly max_speed
    """
    progress = clamp(progress, 0, 1)

    x = progress * 5 - 2.5
    sigmoid = 1 / (1 + math.exp(-x))

    sigmoid_min = 1 / (1 + math.exp(2.5))
    sigmoid_max = 1 / (1 + math.exp(-2.5))

    normalized = (
        (sigmoid - sigmoid_min)
        / (sigmoid_max - sigmoid_min)
    )

    return min_speed + (max_speed - min_speed) * normalized

def _motor_duty(target_speed, motor, direction, prev_err, filt_deriv, dt,
                 kff, kp, kd, deriv_alpha):
    """
    One PD+feedforward step for a single motor.
    target_speed and motor.speed() are both deg/s, direction-normalised so
    'forward' (or 'this motor's intended direction') is always positive.
 
    prev_err=None means "first sample" -> derivative term is skipped instead
    of being computed against an assumed previous error of 0, which would
    otherwise produce a large spike on the very first loop iteration.
 
    Returns (duty, err, filt_deriv) -- feed err/filt_deriv back in as
    prev_err/filt_deriv next call.
    """
    speed = direction * motor.speed()
    err = target_speed - speed
    if prev_err is None:
        deriv = 0
    else:
        deriv = (err - prev_err) / dt * 1000  # deg/s^2
    filt_deriv = deriv_alpha * deriv + (1 - deriv_alpha) * filt_deriv
    duty = kff * target_speed + kp * err + kd * filt_deriv
    return duty, err, filt_deriv

 
def move_distance(target_distance, default_min_speed=20, default_max_speed=1000, default_ramp_dist=100):
    """
    Moves the robot forwards using direct duty-cycle (dc) control instead of motor.run().

    Architecture:
    1. Outer loop (same as before): sigmoid motion profile over distance travelled produces a target speed in deg/s.
    2. Inner loop (new): a PD+feedforward controller turns (target_speed - motor.speed()) into a duty command sent via motor.dc().
       This is the part that replaces Pybricks' internal run() PID with your own,faster, robot-specific one.
    3. Sync loop (upgraded from P-only to PD): keeps motorA/motorB travelling the same distance by trimming duty based on their distance error and its derivative.
    """
    # kff = 0.09        # %/(deg/s)   -- TUNE FIRST, on a full battery
    # kp_speed = 0.05     # %/(deg/s) of speed error
    # kd_speed = 0.004    # %/(deg/s^2) of speed-error rate of change
    # kp_sync = 0.01
    # kd_sync = 0.002
    # deriv_alpha = 0.5

    # ---------- SPEED CONTROLLER ----------

    # Feedforward
    kS = 12.0       # friction feedforward - leave at 0 for now
    kV = 0.08      # speed feedforward
    kA = 0.0       # acceleration feedforward - tune later

    # PID feedback
    kp_speed = 0.05
    ki_speed = 0.8
    kd_speed = 0.0     # disable D while tuning PI first

    # Integral is allowed to contribute at most +/-30% duty
    I_DUTY_LIMIT = 30


    # ---------- WHEEL SYNC CONTROLLER ----------
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
 
    # prev_err_A =  prev_err_B = None
    # filt_deriv_A, filt_deriv_B = 0, 0
    # prev_sync_error = 0
    # filt_sync_deriv = 0

    prev_err_A = prev_err_B = None
    filt_deriv_A, filt_deriv_B = 0, 0

    # Integral state for each motor
    integral_A = 0
    integral_B = 0

    # Used later for acceleration feedforward
    prev_target_speed = None

    prev_sync_error = 0
    filt_sync_deriv = 0
 
    avg_dist = 0

    last_print = 0
    print("moving")
    while avg_dist < target_distance:
        now = timer.time()
        dt = max(now - prev_time, 1)  # ms; guard against divide-by-zero on a fast tick
        prev_time = now
        dt_s = dt / 1000.0 #seconds
 
        # Distances travelled, normalised so "forward" is always positive regardless of direction.
        dist_A = angle_to_dist(direction * motorA.angle())
        dist_B = angle_to_dist(direction * motorB.angle())
        avg_dist = (dist_A + dist_B) / 2
 
        phase, progress = get_phase(avg_dist, ramp_dist, decel_dist, cruise_dist, target_distance)
        if phase == "CRUISE":
            target_speed = max_speed
        elif phase == "RAMP_UP":
            target_speed = compute_ramp_speed(
            progress,
            default_min_speed,
            max_speed
        )
        else:  # RAMP_DOWN

            target_speed = compute_ramp_speed(
                progress,
                0,
                max_speed
        )

        # ---- target acceleration ----
        # This currently doesn't affect duty because kA = 0.
        # We're calculating it now so the controller is ready for kA later.
        if prev_target_speed is None:
            target_accel = 0
        else:
            target_accel = (target_speed - prev_target_speed) / dt_s

        

        prev_target_speed = target_speed
 
        # Faster model-estimated speed for custom control loops
        _, estimated_speed_A, _, _ = motorA.model.state()
        _, estimated_speed_B, _, _ = motorB.model.state()

        speed_A = direction * estimated_speed_A
        speed_B = direction * estimated_speed_B
 
        err_A = target_speed - speed_A
        err_B = target_speed - speed_B
 
        # deg/s^2
        if prev_err_A is None:
            deriv_A = 0
        else:
            deriv_A = (err_A - prev_err_A) / dt * 1000  # deg/s^2

        if prev_err_B is None:
            deriv_B = 0
        else:
            deriv_B = (err_B - prev_err_B) / dt * 1000

        filt_deriv_A = deriv_alpha * deriv_A + (1 - deriv_alpha) * filt_deriv_A
        filt_deriv_B = deriv_alpha * deriv_B + (1 - deriv_alpha) * filt_deriv_B

        prev_err_A, prev_err_B = err_A, err_B

        # ---- integral / accumulated speed error ----
        if ki_speed != 0:

            # Convert our desired maximum I contribution
            # from duty % into the equivalent integral limit.
            integral_limit = I_DUTY_LIMIT / ki_speed

            integral_A += err_A * dt_s
            integral_B += err_B * dt_s

            # Anti-windup
            integral_A = clamp(
                integral_A,
                -integral_limit,
                integral_limit
            )

            integral_B = clamp(
                integral_B,
                -integral_limit,
                integral_limit
            )


        # Integral duty contribution
        i_term_A = ki_speed * integral_A
        i_term_B = ki_speed * integral_B

 
        # ---- feedforward ----

        feedforward = (
            kS
            + kV * target_speed
            + kA * target_accel
        )


        # ---- feedforward + PID feedback ----

        duty_A = (
            feedforward
            + kp_speed * err_A
            + i_term_A
            + kd_speed * filt_deriv_A
        )

        duty_B = (
            feedforward
            + kp_speed * err_B
            + i_term_B
            + kd_speed * filt_deriv_B
        )
 
        # ---- wheel sync PD (was P-only) ----
        sync_error = dist_A - dist_B
        sync_deriv = (sync_error - prev_sync_error) / dt * 1000
        filt_sync_deriv = deriv_alpha * sync_deriv + (1 - deriv_alpha) * filt_sync_deriv
        prev_sync_error = sync_error
 
        sync_correction = kp_sync * sync_error + kd_sync * filt_sync_deriv
 
        duty_A -= sync_correction
        duty_B += sync_correction

        

        motorA.dc(direction * clamp(duty_A, -100, 100))
        motorB.dc(direction * clamp(duty_B, -100, 100))

        if now - last_print >= 50:
            print(
                "target:", round(target_speed),
                "speed:", round(speed_A), round(speed_B),
                "duty:", round(duty_A, 1), round(duty_B, 1),
                "I:", round(i_term_A, 1), round(i_term_B, 1)
            )
            last_print = now
 
        wait(5)
 
    # brake() rather than stop(): with dc()-style control you generally want a firm,
    # repeatable stop rather than a coast that depends on residual momentum/friction.
    motorA.brake()
    motorB.brake()
    ev3.speaker.beep()
    wait(beep_time)