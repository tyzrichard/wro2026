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
    motorC.run_until_stalled(-1000, then=Stop.HOLD, duty_limit=100)

def raise_slapper():
    motorC.run_target(1000, -750) # speed, rot_angle

def grab():
    motorD.run_until_stalled(600, then=Stop.HOLD, duty_limit=90)

def release(wait=True):
    motorD.run_target(300, 0, wait=wait)
    # motorD.run_until_stalled(-300, then=Stop.HOLD, duty_limit=90, wait=wait)

def move(point="left", wait=True):
    if point == "left" or point == 0:
        motorC.run_target(1000, -40, then=Stop.HOLD, wait=False)
    elif point == "mid" or point == 1:
        motorC.run_target(1000, -420, then=Stop.HOLD, wait=False)
    elif point == "right" or point == 2:
        motorC.run_target(1000, -800, then=Stop.HOLD, wait=False)
    elif point == "left_block":
        motorC.run_target(1000, -250, then=Stop.HOLD, wait=wait)
    elif point == "right_block":
        motorC.run_target(1000, -720, then=Stop.HOLD, wait=wait)

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
        acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.move_distance(160*(current - move_to), default_ramp_dist=150) 
        acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        # acc.line_following_blackvar(small=True)
        acc.move_distance(50) # used to be 160
    else: # sector is on right
        acc.turn_degrees(90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        acc.move_distance(160*(move_to - current), default_ramp_dist=150)
        acc.turn_degrees(-90, mode="spot", default_max_speed=1000, default_ramp_dist=100)
        # acc.line_following_blackvar(small=True)
        acc.move_distance(50) # used to be 160

def grab_blocks(sector, placed, start_dist=0, block_dist=60): #old start_dist was 30
    moved = False
    forward_mm = 0
    items = list(placed.items())  
    num_cols = len(items)
    for col_idx, (col, cnt) in enumerate(items):
        for i in range(cnt):
            is_last_block = (col_idx == num_cols - 1) and (i == cnt - 1)
            blockInven[sector] += 1
    
            if blockInven[sector] == 1:
                acc.move_distance(-start_dist, default_ramp_dist=130)
                # move('left_block')
                forward_mm = start_dist
            elif blockInven[sector] == 2:
                if not(moved):
                    acc.move_distance(-start_dist, default_ramp_dist=130)
                move('right_block')
                forward_mm = start_dist
            elif blockInven[sector] == 3:
                if not(moved):
                    acc.move_distance(block_dist - start_dist, default_ramp_dist=130)
                    # move('left_block')
                else:
                    move('left_block', wait=False)
                    acc.move_distance(block_dist, default_ramp_dist=130)
                forward_mm = start_dist + block_dist
            elif blockInven[sector] == 4:
                if not(moved):
                    acc.move_distance(block_dist - start_dist, default_ramp_dist=130)
                move('right_block')
                forward_mm = start_dist + block_dist
            elif blockInven[sector] == 5:
                if not(moved):
                    acc.move_distance(2*block_dist - start_dist, default_ramp_dist=130)
                else:
                    move('left_block', wait=False)
                    acc.move_distance(block_dist, default_ramp_dist=130)
                # move('left_block')
                forward_mm = start_dist + 2*block_dist
            elif blockInven[sector] == 6:
                if not(moved):
                    acc.move_distance(2*block_dist - start_dist, default_ramp_dist=130)
                move('right_block')
                forward_mm = start_dist + 2*block+dist
            grab()
            move(col)
            if is_last_block:
                release(False) # starts to move back while releasing
                return forward_mm
            release()
            moved = True # subsequent grabs factor in the previous moved distance 
    

def entire_block_phase(plan):
    current_sector = 0
    next_sector, next_placed = None, None
    for trip_num, (color, placed) in enumerate(plan, start=1):
        if trip_num < len(plan):
            next_sector, next_placed = plan[trip_num]     
            
        if trip_num == 1:
            current_sector = color
            move('left_block', wait=False)
            if current_sector == 0:
                acc.turn_degrees(-90, mode="spot")
                # acc.line_following(240, sensor=middleColor, default_ramp_dist=130)
                acc.move_distance(240, default_ramp_dist=130)
                acc.turn_degrees(90, mode="spot")
            elif current_sector == 1:
                acc.turn_degrees(-90, mode="spot")
                # acc.line_following(80, sensor=middleColor, default_ramp_dist=130)
                acc.move_distance(80, default_ramp_dist=130)
                acc.turn_degrees(90, mode="spot")
            elif current_sector == 2:
                acc.turn_degrees(90, mode="spot")
                # acc.line_following(80, sensor=middleColor, default_ramp_dist=130)
                acc.move_distance(80, default_ramp_dist=130)
                acc.turn_degrees(-90, mode="spot")
            elif current_sector == 3:
                acc.turn_degrees(90, mode="spot")
                # acc.line_following(240, sensor=middleColor, default_ramp_dist=130)
                acc.move_distance(240, default_ramp_dist=130)
                acc.turn_degrees(-90, mode="spot")
            acc.move_distance(180, default_ramp_dist=130)
            # acc.line_following(160, sensor=middleColor, default_ramp_dist=130)
            # acc.line_following_blackvar(small=True)
            forward_mm = grab_blocks(current_sector, placed)
            acc.move_distance(-(50 + forward_mm), default_ramp_dist=100)
        else:
            move_to_sector(current_sector, color)
            current_sector = color
            forward_mm = grab_blocks(current_sector, placed)
            if next_sector is not None:
                acc.move_distance(-(50 + forward_mm), default_ramp_dist=100)
            else: # move back to mosaic black line
                acc.move_distance(-(160 + forward_mm), default_ramp_dist=100)
                if current_sector == 0:
                    acc.turn_degrees(90, mode="spot")
                    # acc.line_following(240, sensor=middleColor, default_ramp_dist=130)
                    acc.move_distance(240, default_ramp_dist=130)
                    acc.turn_degrees(90, mode="spot")
                elif current_sector == 1:
                    acc.turn_degrees(90, mode="spot")
                    acc.line_following(80, sensor=middleColor, default_ramp_dist=130)
                    # acc.move_distance(80, default_ramp_dist=130)
                    acc.turn_degrees(90, mode="spot")
                elif current_sector == 2:
                    acc.turn_degrees(-90, mode="spot")
                    # acc.line_following(80, sensor=middleColor, default_ramp_dist=130)
                    acc.move_distance(80, default_ramp_dist=130)
                    acc.turn_degrees(-90, mode="spot")
                elif current_sector == 3:
                    acc.turn_degrees(-90, mode="spot")
                    # acc.line_following(240, sensor=middleColor, default_ramp_dist=130)
                    acc.move_distance(240, default_ramp_dist=130)
                    acc.turn_degrees(-90, mode="spot")
                acc.move_distance(300, default_ramp_dist=130)
                acc.line_following_blackvar(kp=0, kd=0)
        # Premptively move to the left/right block while moving to the next sector
        if next_sector is not None:
            if (blockInven[next_sector] + 1) % 2 == 1:
                move('left_block', wait=False)
            else:
                move('right_block', wait=False)


def drop_blocks(target_distance, last_drop_dist=390, drop_interval_dist=80):
    motorC.run_target(1000, -800, then=Stop.HOLD, wait=False)
    acc.move_distance(last_drop_dist, default_ramp_dist=100)
    for i in range(3):
        slap_slapper()
        acc.move_distance(-30, default_ramp_dist=100)
        acc.move_distance(drop_interval_dist, default_ramp_dist=100)
        raise_slapper()
    slap_slapper()
    acc.move_distance(-30, default_ramp_dist=100)
    acc.move_distance(target_distance - last_drop_dist - 3*drop_interval_dist, default_ramp_dist=100)
    raise_slapper()

