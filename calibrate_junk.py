from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from Hardware import motorA, motorB, motorC, motorD, robot, leftColor, middleColor, rightColor, pushButton
import Acceleration as acceleration
import NewAccel as nacc
import MiscSetup as misc
import SlapSorter as slap


ev3 = EV3Brick()

# motorA = Motor(Port.A)
# motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
# motorC = Motor(Port.C)
# motorD = Motor(Port.D)

# leftColor = color.HiTechnicColorV2(Port.S1)
# middleColor = color.HiTechnicColorV2(Port.S2)
# rightColor = color.HiTechnicColorV2(Port.S3)

# pushButton = TouchSensor(Port.S4)

print(ev3.battery.voltage())

acc = acceleration.AccelerationController()

# ---------- MOTOR CALIBRATION SETTINGS ----------

DUTY_LEVELS = [50, 60, 70, 80]
RUNS_PER_DUTY = 3

SETTLE_MS = 500
SAMPLE_MS = 1000

def touch_button_pressed():
    """Return True while the pushbutton is pressed."""
    return pushButton.pressed() 

def wait_for_new_touch_press():
    """Wait for one new press without counting a held button twice."""
    while touch_button_pressed():
        wait(10)

    while not touch_button_pressed():
        wait(10)

    ev3.speaker.beep()
    print("Touch button on Port 4 pressed")

def run_test(duty, run_number, results):
    """
    Applies the same raw duty to both drive motors,
    allows them to settle, then measures encoder speed
    over SAMPLE_MS.
    """

    print(
        "Ready:",
        "duty", duty,
        "run", run_number,
        "- press TOUCH BUTTON"
    )

    # Audible indication that the robot is waiting.
    ev3.speaker.beep()

    wait_for_new_touch_press()

    print("Running...")

    # Apply exactly the same raw duty to both motors.
    motorA.dc(duty)
    motorB.dc(duty)

    # Allow speed to settle before measurement.
    wait(SETTLE_MS)

    start_A = motorA.angle()
    start_B = motorB.angle()

    # Measure over a relatively long interval to reduce
    # encoder quantisation/noise.
    wait(SAMPLE_MS)

    end_A = motorA.angle()
    end_B = motorB.angle()

    motorA.brake()
    motorB.brake()

    speed_A = (end_A - start_A) * 1000.0 / SAMPLE_MS
    speed_B = (end_B - start_B) * 1000.0 / SAMPLE_MS

    if speed_A != 0:
        ratio = speed_B / speed_A
    else:
        ratio = 0

    line = (
        "CAL duty: {} run: {} A: {:.1f} B: {:.1f} B/A: {:.4f}\n"
        .format(
            duty,
            run_number,
            speed_A,
            speed_B,
            ratio
        )
    )

    print(line.strip())

    results.append(
        (
            duty,
            run_number,
            speed_A,
            speed_B,
            ev3.battery.voltage()
        )
    )
    # Two beeps = test completed.
    ev3.speaker.beep()
    wait(100)
    ev3.speaker.beep()

    wait(500)

def print_calibration(results):

    print("----- MOTOR CALIBRATION -----")

    for duty, run_number, speed_A, speed_B, battery in results:

        ratio = speed_B / speed_A if speed_A else 0

        print(
            "CAL duty:", duty,
            "run:", run_number,
            "A:", round(speed_A, 1),
            "B:", round(speed_B, 1),
            "B/A:", round(ratio, 4),
            "battery:", battery
        )

    print("-----------------------------")

if ev3.battery.voltage() >= 7000:
    results = []
    try:
        # Testing: Print White Values of all 3 color sensors
        # while True:
        #     print(leftColor.read('RGB')[-1] + " " + middleColor.read('RGB')[-1] + " " + rightColor.read('RGB')[-1])
        
        # Testing: Move Slider
        # while True:
        #     slap.move("left_block")
        #     slap.slap_slapper()
        

        print("Motor calibration")
        print("Results will be saved to:", results)
        print("Press the touch button on Port 4 before every run.")


        try:
            for duty in DUTY_LEVELS:

                for run_number in range(1, RUNS_PER_DUTY + 1):
                    run_test(
                        duty,
                        run_number,
                        results
                    )

                ev3.speaker.beep(800, 400)
                wait(500)

        finally:
            motorA.brake()
            motorB.brake()
        print("Calibration complete.")
        print("Saved to:", results)

        ev3.speaker.beep(1000, 700)

        print("Calibration complete.")
        print("Reconnect laptop, then press touch button.")

        # Distinctive completion beep.
        ev3.speaker.beep(1000, 700)

        # Wait here while you reconnect USB.
        wait_for_new_touch_press()

        # Now dump everything to the terminal.
        print_calibration(results)

        ev3.speaker.beep(1200, 500)

    
        
        while True:
            wait_for_new_touch_press()
            nacc.move_distance(100)
        # wait_for_new_touch_press()

        # for i in [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]:
        #     motorA.dc(i)
        #     motorB.dc(i)
        #     wait(1000)

        #     speed_A = motorA.speed()
        #     speed_B = motorB.speed()

        #     ratio_A = i / speed_A if speed_A != 0 else "stalled"
        #     ratio_B = i / speed_B if speed_B != 0 else "stalled"

        #     print("A", i, speed_A, ratio_A)
        #     print("B", i, speed_B, ratio_B)

        # motorA.brake()
        # motorB.brake()
        # 0. Setup
        # ev3.speaker.beep()
        # wait_for_new_touch_press()
        # motorD.hold()
        # misc.reset_slider("right",False)
        # wait_for_new_touch_press()    

        # # 1. Move and get Yellows
        # slap.move("left_block")
        # acc.move_distance(220, default_ramp_dist=150)
        # # acc.turn_degrees(-180, mode="arc", turn_radius=138, default_ramp_dist=150)
        # acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=250)
        # acc.move_distance(270, default_ramp_dist=150)
        # acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=250)
        # acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
        # acc.line_following_blackvar(small=True)
        # acc.move_distance(-25, default_ramp_dist=130)
        # slap.grab()
        # slap.move("left")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release(wait=False)
        # acc.move_distance(63, default_max_speed=600)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("left")
        # slap.release(wait=False)
        # acc.move_distance(64, default_max_speed=600)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("left_block")
        # acc.move_distance(-200)

        # # 2. Move and get Blues
        # acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=225)
        # acc.move_distance(160, default_ramp_dist=250) 
        # acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=225)
        # acc.line_following(70, default_ramp_dist=200, sensor=middleColor) 
        # acc.line_following_blackvar(small=True)
        # acc.move_distance(-27, default_max_speed=600, default_ramp_dist=130)
        # slap.grab()
        # slap.move("left")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release(wait=False)
        # acc.move_distance(63, default_max_speed=600)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("left")
        # slap.release(wait=False)
        # acc.move_distance(64, default_max_speed=600)
        # slap.move("left_block")
        # slap.grab()
        # slap.move("mid")
        # slap.release()
        # slap.move("right_block")
        # slap.grab()
        # slap.move("right")
        # slap.release()
        # acc.move_distance(30, default_max_speed=600)
        # acc.turn_degrees(90, mode="spot", default_ramp_dist=200)
        # acc.move_distance(360, default_ramp_dist=100)
        # acc.move_distance(-240, default_ramp_dist=100)
        # acc.turn_degrees(90, mode="spot", default_ramp_dist=400)
        

        # # 3. Throw all blocks onto mosaic and pray it works
        # # acc.turn_degrees(180, mode="spot")
        # # acc.turn_degrees(-80, mode="arc", turn_radius=50, default_ramp_dist=160)
        # # acc.turn_degrees(80, mode="arc", turn_radius=53, default_ramp_dist=160)
        # acc.move_distance(300, default_ramp_dist=150)
        # acc.line_following(350, default_ramp_dist=150, sensor=middleColor)
        # acc.line_following_blackvar()
        # slap.drop_blocks(1300)

        # # 4. Slap them greens
        # acc.line_following_blackvar()
        # acc.move_distance(-70,default_ramp_dist=100)
        # slap.slap_slapper()
        # acc.turn_degrees(180, mode="spot", default_ramp_dist=175)
        # acc.move_distance(330, default_ramp_dist=100)
        # slap.raise_slapper(wait=True)
        # acc.move_distance(-170, default_ramp_dist=100)
        # slap.slap_slapper()
        # acc.move_distance(185, default_ramp_dist=100)

        # # 5. ram the rest if got time
        # slap.raise_slapper(wait=True)
        # acc.move_distance(-400)
        # acc.turn_degrees(-90, mode="arc", turn_radius=150, default_ramp_dist=100)
        # acc.move_distance(270, default_ramp_dist=100)
        # acc.turn_degrees(90, mode="arc", turn_radius=150, default_ramp_dist=100)
        # # slap.slap_slapper()
        # acc.move_distance(750,default_ramp_dist=0)
        # # acc.turn_degrees(25, mode="spot", default_ramp_dist=160)
        # # acc.move_distance(150, default_ramp_dist=50)
        # # acc.turn_degrees(-25, mode="spot", default_ramp_dist=160)
        # # acc.move_distance(200, default_ramp_dist=50)
    
    except Exception as e:
        print(f"An error occurred: {e}")
        raise