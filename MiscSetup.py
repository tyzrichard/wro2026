from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Button, Direction
from pybricks.tools import wait, StopWatch, DataLog

import SlapSorter as slap

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)
motorD = Motor(Port.D)

def reset_slider(point, wait_logic):
    """
    Resets arm slider position and angle
    """
    motorD.reset_angle(0)

    # Home motor C against a physical stall point
    motorC.run(1000)
    while not motorC.control.stalled():
        wait(10)
    motorC.stop()
    motorC.reset_angle(0)
    motorC.run_target(100, -600, then=Stop.HOLD, wait=False)

    slap.move(point, wait_logic)