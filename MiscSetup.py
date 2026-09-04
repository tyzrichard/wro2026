from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Button, Direction
from pybricks.tools import wait, StopWatch, DataLog

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)

def reset_slider():
    """
    Resets arm slider position and angle
    """
    motorC.run(300)
    while not motorC.control.stalled():
        wait(10)
    motorC.stop()
    motorC.reset_angle(0)
    motorC.run_target(900, -1300, then=Stop.HOLD)

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

def calibrate_colours():
    left_sensor = Ev3devSensor(Port.S1)
    middle_sensor = Ev3devSensor(Port.S2)
    right_sensor = Ev3devSensor(Port.S3)
    
    ev3.screen.clear()
    
    while True:
        ev3.screen.clear()
        ev3.screen.print(left_sensor.read('RGB'))
        ev3.screen.print(middle_sensor.read('RGB'))
        ev3.screen.print(right_sensor.read('RGB'))
        wait(100)

def dancing():
    # why is this a thing.
    acc.move_distance(200)
    acc.turn_degrees(-45, default_max_speed=500)
    acc.turn_degrees(90, default_max_speed=1000)
    acc.turn_degrees(-45, default_max_speed=500)
    acc.turn_degrees(360, default_max_speed=1000)