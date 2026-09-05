#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
import Acceleration as acceleration
import MiscSetup as misc

motorC = Motor(Port.C)
motorD = Motor(Port.D)

def slap_slapper():
    # Runs until it physically can't move any further (hits the limit), then stops
    motorC.run_until_stalled(-1000, then=Stop.HOLD, duty_limit=90)

def raise_slapper():
    motorC.run_angle(600, 350) # speed, rot_angle

def grab():
    motorD.run_until_stalled(600, then=Stop.HOLD, duty_limit=90)

def slight_raise():
    motorD.run_angle(300, -60)

def release():
    motorD.run_until_stalled(-200, then=Stop.HOLD, duty_limit=90)

def left():
    motorC.run_target(2000, -100, then=Stop.HOLD, wait=False)

def mid():
    motorC.run_target(2000, -650, then=Stop.HOLD, wait=False)

def right():
    motorC.run_target(2000, -1250, then=Stop.HOLD, wait=False)

def left_block():
    motorC.run_target(2000, -300, then=Stop.HOLD)

def right_block():
    motorC.run_target(2000, -1150, then=Stop.HOLD)

def order_blocks(fill_order):
    n = len(fill_order)
    lengths = tuple(len(c) for c in fill_order)
    target = lengths

    def frontier(ptrs):
        colors = set()
        for i in range(n):
            if ptrs[i] < lengths[i]:
                colors.add(fill_order[i][ptrs[i]])
        return colors

    def advance_color(ptrs, color):
        ptrs = list(ptrs)
        changed = True
        while changed:
            changed = False
            for i in range(n):
                if ptrs[i] < lengths[i] and fill_order[i][ptrs[i]] == color:
                    ptrs[i] += 1
                    changed = True
        return tuple(ptrs)

    memo = {}
    choice = {}

    def dp(ptrs):
        if ptrs == target:
            return 0
        if ptrs in memo:
            return memo[ptrs]
        best, best_color = None, None
        for color in frontier(ptrs):
            new_ptrs = advance_color(ptrs, color)
            cost = 1 + dp(new_ptrs)
            if best is None or cost < best:
                best, best_color = cost, color
        memo[ptrs] = best
        choice[ptrs] = best_color
        return best

    start = tuple([0] * n)
    total_trips = dp(start)

    # reconstruct the actual plan
    plan = []
    ptrs = start
    while ptrs != target:
        color = choice[ptrs]
        new_ptrs = advance_color(ptrs, color)
        placed = {i: new_ptrs[i] - ptrs[i] for i in range(n) if new_ptrs[i] != ptrs[i]}
        plan.append((color, placed))
        ptrs = new_ptrs

    print(total_trips)
    print(plan)
    return total_trips, plan


