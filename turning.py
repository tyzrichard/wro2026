# file: robot_turn.py

from math import pi
from pybricks.ev3devices import Motor
from pybricks.parameters import Stop

# Robot constants (measure your robot!)
R = 0.031   # wheel radius [m]
L = 0.192   # track width [m]
MOTOR_MAX_DPS = 800
R_MIN = 0.08   # [m] arc radius for full steer (tune)

def linear_to_deg(distance_m):
    """Convert linear distance in meters to wheel rotation in degrees."""
    return (distance_m / (2 * pi * R)) * 360.0

def turn_degrees(left_motor: Motor, right_motor: Motor, angle_deg: float,
                 mode="arc", forward_dist=1.5, speed_dps=600):
    """
    Turn the robot by a given angle in degrees.
    Differential drive: both motors run concurrently.
    left_motor, right_motor : Motor objects
    angle_deg : robot rotation in degrees (+ = right, - = left)
    mode : "pivot" (in place) or "arc" (car-like)
    forward_dist : [m] distance travelled along arc (only for mode="arc")
    speed_dps : motor speed [deg/s]
    """

    if mode == "pivot":
        # Each wheel rotation needed (deg)
        wheel_deg = abs(angle_deg) * (L / (2 * R))
        sgn = 1 if angle_deg > 0 else -1
        # Run wheels equal & opposite concurrently
        left_motor.run_angle(-sgn * speed_dps, wheel_deg, Stop.BRAKE, False)
        right_motor.run_angle(sgn * speed_dps, wheel_deg, Stop.BRAKE, True)

    elif mode == "arc":
        # Pick arc radius depending on requested turn
        s = angle_deg / 90.0
        Rturn = max(R_MIN, R_MIN / abs(s))  # bigger radius for smaller angle

        # Arc length robot center travels
        arc_length = (abs(angle_deg) * pi/180) * Rturn

        # Outer/inner wheel distances
        if s > 0:
            # Right turn: left wheel travels more
            dist_L = arc_length * (Rturn + L/2) / Rturn
            dist_R = arc_length * (Rturn - L/2) / Rturn
        else:
            # Left turn: right wheel travels more
            dist_R = arc_length * (Rturn + L/2) / Rturn
            dist_L = arc_length * (Rturn - L/2) / Rturn

        # Convert to wheel degrees
        deg_R = linear_to_deg(dist_R)
        deg_L = linear_to_deg(dist_L)

        right_motor.run_angle(speed_dps, deg_R, Stop.BRAKE, False)
        left_motor.run_angle(speed_dps, deg_L, Stop.BRAKE, True)

    else:
        raise ValueError("mode must be 'pivot' or 'arc'")
