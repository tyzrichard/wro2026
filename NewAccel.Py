from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button
from Hardware import motorA, motorB, motorC, motorD, robot, leftColor, middleColor, rightColor, pushButton
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from LineTracer import PDController
import math
import Logger as logger

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

def dist_planning(target_distance, default_max_speed, default_ramp_dist, decel_scale=1.15):
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

 
def move_distance(target_distance, default_min_speed=160, default_max_speed=1000, default_ramp_dist=80):
    """
    Moves the robot forwards using direct duty-cycle (dc) control instead of motor.run().

    Architecture:
    1. Outer loop (same as before): sigmoid motion profile over distance travelled produces a target speed in deg/s.
    2. Inner loop (new): a PD+feedforward controller turns (target_speed - motor.speed()) into a duty command sent via motor.dc().
       This is the part that replaces Pybricks' internal run() PID with your own,faster, robot-specific one.
    3. Sync loop (upgraded from P-only to PD): keeps motorA/motorB travelling the same distance by trimming duty based on their distance error and its derivative.
    """

    # ---------- SPEED CONTROLLER ----------

    # Feedforward
    kS = 28.0       # friction feedforward - leave at 0 for now
    #kV = 0.10      # speed feedforward
    kV_A = 0.100
    kV_B = 0.0965
    #kA = 0.003 :SOB: it was nice while you lasted      # acceleration feedforward - tune later
    kA_accel = 0.004
    kA_decel = 0.003

    MAX_TARGET_ACCEL = 3000

    MAX_CRUISE_SPEED = 620

    RAMP_DOWN_KS_MIN = 18.0

    # PID feedback
    kp_speed = 0.05
    ki_speed = 0.8
    kd_speed = 0.0     # disable D while tuning PI first

    # Integral settings
    I_DUTY_LIMIT = 15
    I_ACTIVE_ERROR = 100

    # ---------- FINAL POSITION SETTLING ----------
    DISTANCE_SCALE = 1.03
    SETTLE_ZONE = 12.0        # mm from destination to enter position control
    SETTLE_TOL = 1.0       # acceptable final position error, mm

    SETTLE_KP = 20.0          # target deg/s per mm of position error
    SETTLE_MAX_SPEED = 60    # maximum correction speed, deg/s
    SETTLE_MIN_SPEED = 50

    SETTLE_KS = 40.0

    SETTLE_STABLE_CYCLES = 3  # must stay inside tolerance this many loops


    # ---------- WHEEL SYNC CONTROLLER ----------
    kp_sync = 0.9
    kd_sync = 0.01

    MAX_SYNC_CORRECTION = 6.0

    deriv_alpha = 0.5
    target_distance *= DISTANCE_SCALE
    
    ramp_dist, decel_dist, cruise_dist, max_speed = dist_planning(abs(target_distance), default_max_speed, default_ramp_dist)
 
    # motorA.reset_angle(0)
    # motorB.reset_angle(0)

    max_speed = min(max_speed, MAX_CRUISE_SPEED)

    start_angle_A = motorA.angle()
    start_angle_B = motorB.angle()

    first_move_A = None
    first_move_B = None
    first_move_printed = False
 
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
    prev_target_speed = 0.0

    prev_sync_error = 0
    filt_sync_deriv = 0
 
    avg_dist = 0

        
    # Final position controller state
    settling = False
    settled_cycles = 0

    last_print = 0
    logger.log_print("moving")
    while True: # avg_dist < target_distance this is to let the robot correct itself if it overshoots the target distance
        now = timer.time()
        dt = max(now - prev_time, 1)  # ms; guard against divide-by-zero on a fast tick
        prev_time = now
        dt_s = dt / 1000.0 #seconds
 
        # Distances travelled, normalised so "forward" is always positive regardless of direction.
        angle_A = motorA.angle() - start_angle_A
        angle_B = motorB.angle() - start_angle_B

        # Measure actual startup timing from encoder movement.
        if first_move_A is None and abs(angle_A) >= 1:
            first_move_A = now

        if first_move_B is None and abs(angle_B) >= 1:
            first_move_B = now

        if (
            not first_move_printed
            and first_move_A is not None
            and first_move_B is not None
        ):
            logger.log_print(
                "START A:", first_move_A,
                "B:", first_move_B,
                "difference:", first_move_A - first_move_B,
                "ms"
            )

            first_move_printed = True

        dist_A = angle_to_dist(direction * angle_A)
        dist_B = angle_to_dist(direction * angle_B)

        
        avg_dist = (dist_A + dist_B) / 2
 
        # Signed position error.
        # Positive = still before target.
        # Negative = overshot target.
        position_error = target_distance - avg_dist


        # Once we enter the settle zone, stay in settle mode.
        if not settling and position_error <= SETTLE_ZONE:
            settling = True

            # Clear velocity-controller integral history
            # before precision positioning.
            integral_A = 0
            integral_B = 0


        if settling:

            phase = "SETTLE"

            # We are close enough to the requested position.
            if abs(position_error) <= SETTLE_TOL:

                motorA.brake()
                motorB.brake()

                integral_A = 0
                integral_B = 0

                settled_cycles += 1

                if settled_cycles >= SETTLE_STABLE_CYCLES:
                    break

                wait(5)
                continue


            # Not inside tolerance anymore.
            settled_cycles = 0


            # Convert position error into signed target speed.
            #
            # Before target  -> positive target speed
            # Past target    -> negative target speed
            target_speed = SETTLE_KP * position_error

            target_speed = clamp(
                target_speed,
                -SETTLE_MAX_SPEED,
                SETTLE_MAX_SPEED
            )

            # 20 ms motor.speed() resolution is roughly 50 deg/s,
            # so don't ask the velocity loop to regulate below that.
            if 0 < target_speed < SETTLE_MIN_SPEED:
                target_speed = SETTLE_MIN_SPEED

            elif -SETTLE_MIN_SPEED < target_speed < 0:
                target_speed = -SETTLE_MIN_SPEED


        else:

            # Normal distance-based motion profile
            phase, progress = get_phase(
                avg_dist,
                ramp_dist,
                decel_dist,
                cruise_dist,
                target_distance
            )

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

 
        # Faster model-estimated speed for custom control loops
        # _, estimated_speed_A, _, _ = motorA.model.state()
        # _, estimated_speed_B, _, _ = motorB.model.state()

        # speed_A = direction * estimated_speed_A
        # speed_B = direction * estimated_speed_B

        # Real measured motor speed.
        # A 50 ms window is slower than model.state(), but gives us
        # trustworthy feedback while tuning the physical drivetrain.
        speed_A = direction * motorA.speed(window=20)
        speed_B = direction * motorB.speed(window=20)
        avg_speed = (speed_A + speed_B) / 2

        if phase == "SETTLE":

            if target_speed < 0 and avg_speed > 50:
                target_speed = 0

            elif target_speed > 0 and avg_speed < -50:
                target_speed = 0

        # ---- target acceleration ----
        # This currently doesn't affect duty because kA = 0.
        # We're calculating it now so the controller is ready for kA later.
        target_accel = (target_speed - prev_target_speed) / dt_s

        target_accel = clamp(
            target_accel,
            -MAX_TARGET_ACCEL,
            MAX_TARGET_ACCEL
        )
    
        prev_target_speed = target_speed

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
        if phase == "SETTLE":

            # Don't use velocity integral during final position settling.
            integral_A = 0
            integral_B = 0

        elif ki_speed != 0:

            # Maximum stored integral based on how much
            # duty we allow the I term to contribute.
            integral_limit = I_DUTY_LIMIT / ki_speed

            # Only integrate relatively small persistent errors.
            #
            # If the error is huge because the robot is accelerating,
            # we DON'T want the integral building up aggressively.
            #
            # Large acceleration errors should eventually be handled
            # by kA, while P handles the immediate error.


            # Accumulate I normally when the error is reasonably small.
            #
            # BUT always allow the integral to unwind when the error
            # is opposite to the existing integral.
            if (
                abs(err_A) < I_ACTIVE_ERROR
                or err_A * integral_A < 0
            ):
                integral_A += err_A * dt_s

            if (
                abs(err_B) < I_ACTIVE_ERROR
                or err_B * integral_B < 0
            ):
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


        # Convert stored integral into duty %
        i_term_A = ki_speed * integral_A
        i_term_B = ki_speed * integral_B

 
        # ---- feedforward ----

        if target_accel >= 0:
            accel_ff = kA_accel * target_accel
        else:
            accel_ff = kA_decel * target_accel

        base_ff_A = (
            kV_A * target_speed
            + accel_ff
        )

        base_ff_B = (
            kV_B * target_speed
            + accel_ff
        )


        # During ramp-down / position settling:
        # only apply kS if the motor is failing to reach
        # the requested speed.
        if phase == "SETTLE":

            # During precision positioning, always provide enough
            # duty to overcome drivetrain friction.
                    
            if target_speed > 0:

                if 0 <= avg_speed < target_speed:
                    friction_ff = SETTLE_KS
                else:
                    friction_ff = 0

            elif target_speed < 0:

                if target_speed < avg_speed <= 0:
                    friction_ff = -SETTLE_KS
                else:
                    friction_ff = 0

            else:
                friction_ff = 0

        elif phase == "RAMP_DOWN":
            current_kS = max(
                kS * progress,
                RAMP_DOWN_KS_MIN
            )

            if target_speed > 0:
                friction_ff = current_kS

            elif target_speed < 0:
                friction_ff = -current_kS

            else:
                friction_ff = 0

        else:
            # Normal ramp-up / cruise
            friction_ff = kS


        feedforward_A = base_ff_A + friction_ff
        feedforward_B = base_ff_B + friction_ff
        # ---- feedforward + PID feedback ----

        duty_A = (
            feedforward_A
            + kp_speed * err_A
            + i_term_A
            + kd_speed * filt_deriv_A
        )

        duty_B = (
            feedforward_B
            + kp_speed * err_B
            + i_term_B
            + kd_speed * filt_deriv_B
        )

        # Reserve 6% duty for wheel-sync correction.
        # Otherwise both motors can saturate at 100% and the
        # sync controller loses all steering authority.
        SYNC_DUTY_LIMIT = 100 - MAX_SYNC_CORRECTION

        duty_A = clamp(
            duty_A,
            -SYNC_DUTY_LIMIT,
            SYNC_DUTY_LIMIT
        )

        duty_B = clamp(
            duty_B,
            -SYNC_DUTY_LIMIT,
            SYNC_DUTY_LIMIT
)
 
        # ---- wheel sync PD (was P-only) ----
        sync_error = dist_A - dist_B
        sync_deriv = (sync_error - prev_sync_error) / dt * 1000
        filt_sync_deriv = deriv_alpha * sync_deriv + (1 - deriv_alpha) * filt_sync_deriv
        prev_sync_error = sync_error
 
        sync_correction = (
            kp_sync * sync_error
            + kd_sync * filt_sync_deriv
        )

        sync_correction = clamp(
            sync_correction,
            -MAX_SYNC_CORRECTION,
            MAX_SYNC_CORRECTION
        )
 
        duty_A -= sync_correction
        duty_B += sync_correction

        

        motorA.dc(direction * clamp(duty_A, -100, 100))
        motorB.dc(direction * clamp(duty_B, -100, 100))

        if now - last_print >= 50:
            logger.log_print(
                "phase:", phase,
                "error:", round(position_error, 2),
                "target:", round(target_speed),
                "speed:", round(speed_A), round(speed_B),
                "sync:", round(sync_error, 2),
                "corr:", round(sync_correction, 2),
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


