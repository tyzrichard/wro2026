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
leftColor = Ev3devSensor(Port.S1)
middleColor = Ev3devSensor(Port.S2)
rightColor = Ev3devSensor(Port.S3)
acc = acceleration.AccelerationController(Kp=0.9)
blockInven = [0, 0, 0, 0]

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

def move(point="left"):
    if point == "left" or point == 0:
        motorC.run_target(2000, -100, then=Stop.HOLD, wait=False)
    elif point == "mid" or point == 1:
        motorC.run_target(2000, -650, then=Stop.HOLD, wait=False)
    elif point == "right" or point == 2:
        motorC.run_target(2000, -1250, then=Stop.HOLD, wait=False)
    elif point == "left_block":
        motorC.run_target(2000, -300, then=Stop.HOLD)
    elif point == "right_block":
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

    return total_trips, plan

def print_plan(plan):
    for trip_num, (color, placed) in enumerate(plan, start=1):
        parts = ["Column %d x%d" % (i + 1, cnt) for i, cnt in placed.items()]
        print("Trip %d: get Sector %s blocks -> place into: %s" % (trip_num, color, ", ".join(parts)))

def move_to_sector(current, move_to):
    if move_to < current: # sector is on left
        # acc.move_distance(-50, default_ramp_dist=100) # to replace
        acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.move_distance(160*(current - move_to), default_ramp_dist=150) 
        acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.line_following_blackvar(small=True)
    else: # sector is on right
        # acc.move_distance(-50, default_ramp_dist=100) # to replace
        acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.move_distance(160*(move_to - current), default_ramp_dist=150)
        acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.line_following_blackvar(small=True)

def grab_blocks(sector, placed):
    moved = False
    forward_mm = 0
    for col, cnt in placed.items():
        for i in range(cnt):
            blockInven[sector] += 1
            if blockInven[sector] == 1:
                acc.move_distance(-20)
                move('left_block')
                forward_mm = 20
            elif blockInven[sector] == 2:
                if not(moved): 
                    acc.move_distance(-20)
                move('right_block')
                forward_mm = 20
            elif blockInven[sector] == 3:
                if not(moved):
                    acc.move_distance(40)
                else:
                    acc.move_distance(60)
                move('left_block')
                forward_mm = 80
            elif blockInven[sector] == 4:
                if not(moved):
                    acc.move_distance(40)
                move('right_block')
                forward_mm = 80
            elif blockInven[sector] == 5:
                if not(moved):
                    acc.move_distance(100)
                else:
                    acc.move_distance(60)
                move('left_block')
                forward_mm = 140
            elif blockInven[sector] == 6:
                if not(moved):
                    acc.move_distance(100)
                move('right_block')
                forward_mm = 140
            grab()
            move(col)
            release()
            moved = True # subsequent grabs factor in the previous moved distance 
    return forward_mm

def entire_block_phase(plan):
    current_sector = 0
    for trip_num, (color, placed) in enumerate(plan, start=1):
        if trip_num == 1:
            current_sector = color
            if current_sector == 0:
                acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
                acc.line_following(240, sensor=middleColor)
                acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
            elif current_sector == 1:
                acc.turn_degrees(90, mode="spot")
                acc.line_following(80, sensor=middleColor)
                acc.turn_degrees(90, mode="spot")
            elif current_sector == 2:
                acc.turn_degrees(-90, mode="spot")
                acc.line_following(80, sensor=middleColor)
                acc.turn_degrees(-90, mode="spot")
            elif current_sector == 3:
                acc.turn_degrees(-90, mode="spot")
                acc.line_following(240, sensor=middleColor)
                acc.turn_degrees(-90, mode="spot")
            acc.line_following(160, sensor=middleColor)
            acc.line_following_blackvar(small=True)
        else:
            move_to_sector(current_sector, color)
            forward_mm = grab_blocks(current_sector, placed)
            acc.move_distance(-(50 + forward_mm))
            current_sector = color