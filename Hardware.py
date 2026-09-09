from pybricks.ev3devices import Motor, TouchSensor
from pybricks.hubs import EV3Brick
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Stop, Button, Direction
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import I2CDevice

ev3 = EV3Brick()

class HiTechnicColorV2:
    def __init__(self, port):
        self.device = I2CDevice(
            port,
            address=0x01,
            custom=True,
            nxt_quirk=True,
        )

        # Active illumination mode.
        self.device.write(0x41, b"\x00")
        wait(100)

    def read(self, mode="RGB"):

        if mode == "RGB":
            data = self.device.read(0x42, 5)

            return (
                data[1],   # Red
                data[2],   # Green
                data[3],   # Blue
                data[4],   # White
            )

        if mode == "COLOR":
            data = self.device.read(0x42, 1)
            return (data[0],)

        raise ValueError("Unsupported mode: " + mode)

motorA = Motor(Port.A)
motorB = Motor(Port.B, Direction.COUNTERCLOCKWISE)
motorC = Motor(Port.C)
motorD = Motor(Port.D)

robot = DriveBase(
    motorA,
    motorB,
    wheel_diameter=62.4,
    axle_track=192,
)

leftColor = HiTechnicColorV2(Port.S1)
middleColor = HiTechnicColorV2(Port.S2)
rightColor = HiTechnicColorV2(Port.S3)

pushButton = TouchSensor(Port.S4)