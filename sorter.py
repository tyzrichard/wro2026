"""
WRO 2026 RoboMission Senior - drop-in sorting section.

TEAM MAIN CODE
    colour_array = scan_all_12_colours()
    move_robot_to_start_point()
    sorting_algorithm(colour_array)
    # Robot is back at the start point. Continue the team's hard-coded route.

The only public function needed by the team's main program is:
    sorting_algorithm(colour_array)

The function does NOT release the three completed slots. It starts at the blue
star, collects all 12 cubes into the onboard FIFO slots, and returns to the star.
The ending heading is not controlled because the team confirmed it does not
matter.
"""


# try:
#     from pybricks.hubs import EV3Brick
#     from pybricks.ev3devices import Motor, ColorSensor
#     from pybricks.parameters import Port, Stop
#     from pybricks.tools import wait

#     ev3 = EV3Brick()
#     right_drive_motor = Motor(Port.A)
#     left_drive_motor = Motor(Port.B)
#     motor_c = Motor(Port.C)
#     motor_d = Motor(Port.D)
#     left_line_sensor = ColorSensor(Port.S1)
#     centre_colour_sensor = ColorSensor(Port.S2)
#     right_line_sensor = ColorSensor(Port.S3)
#     RUNNING_ON_EV3 = True
# except ImportError:
#     # Allows algorithm simulation on a normal computer.
#     RUNNING_ON_EV3 = False


def sorting_algorithm(colour_array):
    """Repair, plan, collect, sort, and return to the start point.

    Input example:
        colour_array = [
            ["Blue", "Green", "Yellow"],
            ["White", "Blue", "Green"],
            ["Yellow", "Green", "Green"],
            ["Blue", "Yellow", "Yellow"],
        ]

    This is deliberately one externally visible function. Its smaller helpers
    are nested inside it so the team only inserts one call into the main route.
    """

    # -----------------------------------------------------------------------
    # DENZEL: CALIBRATION AND MOVEMENT PLACEHOLDERS
    # Ctrl+F any capitalised name below when inserting measured values.
    # -----------------------------------------------------------------------

    SIMULATION_MODE = True  # DENZEL: Change to False after all movement is added.

    BRICK_SPACING_MM = 32
    MAXIMUM_PER_COLOUR = 6

    # Motor C slides to either a supply row or one of the three onboard slots.
    MOTOR_C_SPEED = None                 # DENZEL: Motor C tested speed.
    MOTOR_C_PICKUP_ROW_1 = None          # DENZEL: Motor C position for supply Row 1.
    MOTOR_C_PICKUP_ROW_2 = None          # DENZEL: Motor C position for supply Row 2.
    MOTOR_C_DEPOSIT_SLOT_1 = None        # DENZEL: Motor C position over Slot 1.
    MOTOR_C_DEPOSIT_SLOT_2 = None        # DENZEL: Motor C position over Slot 2.
    MOTOR_C_DEPOSIT_SLOT_3 = None        # DENZEL: Motor C position over Slot 3.

    # Motor D chomps the cube and flings it after Motor C selects a slot.
    MOTOR_D_NORMAL_SPEED = None          # DENZEL: Motor D opening/chomping speed.
    MOTOR_D_FLING_SPEED = None           # DENZEL: Motor D fling speed.
    MOTOR_D_OPEN_POSITION = None         # DENZEL: Motor D fully open position.
    MOTOR_D_CHOMP_POSITION = None        # DENZEL: Motor D cube-holding position.
    MOTOR_D_FLING_POSITION = None        # DENZEL: Motor D end of fling position.

    # Two-sensor line tracing using Sensors 1 and 3.
    BLACK_THRESHOLD = None               # DENZEL: Measured black reflection threshold.
    LINE_BASE_SPEED = None               # DENZEL: Tested line-tracing speed.
    LINE_KP = None                       # DENZEL: Tested proportional gain.
    JUNCTION_OVERSHOOT_MM = None         # DENZEL: Sensor-to-turn-centre offset.
    TURN_INTO_SEGMENT_ANGLE = None       # DENZEL: Turn required at a colour segment.

    # Relative route positions are used only to choose the fastest colour order.
    # The star is between green and blue in the supplied drawing.
    STAR = "Star"
    ROUTE_POSITION = {
        "White": -3,                    # DENZEL: Adjust if measured route differs.
        "Green": -1,                    # DENZEL: Adjust if measured route differs.
        STAR: 0,
        "Blue": 1,                      # DENZEL: Adjust if measured route differs.
        "Yellow": 3,                    # DENZEL: Adjust if measured route differs.
    }

    VALID_COLOURS = ("White", "Green", "Blue", "Yellow")

    # -----------------------------------------------------------------------
    # Scan repair: invalid values and colour counts above six
    # -----------------------------------------------------------------------

    def normalise_colour(value):
        if isinstance(value, str):
            cleaned = value.strip().lower()
            for valid_colour in VALID_COLOURS:
                if cleaned == valid_colour.lower():
                    return valid_colour
        return None

    def pseudo_random_index(option_count, seed_value):
        """Use hub randomness when available; deterministic fallback otherwise."""
        try:
            import urandom
            return urandom.getrandbits(16) % option_count
        except (ImportError, AttributeError):
            # Works in desktop simulation and remains repeatable for debugging.
            mixed = (seed_value * 1103515245 + 12345) & 0x7fffffff
            return mixed % option_count

    def repair_colour_array(raw_array):
        """Return a full 4x3 array in which no colour occurs more than six times."""
        repaired = [[None, None, None] for _ in range(4)]
        counts = {colour: 0 for colour in VALID_COLOURS}
        corrections = []

        # Keep valid readings until that colour reaches its physical limit.
        # Missing cells, invalid readings, and seventh+ occurrences become gaps.
        for row_index in range(4):
            for column_index in range(3):
                raw_value = None
                try:
                    raw_value = raw_array[row_index][column_index]
                except (IndexError, TypeError):
                    pass

                colour = normalise_colour(raw_value)
                if colour is not None and counts[colour] < MAXIMUM_PER_COLOUR:
                    repaired[row_index][column_index] = colour
                    counts[colour] += 1
                else:
                    if colour is None:
                        reason = "failed or unknown scan"
                    else:
                        reason = "{} exceeded six".format(colour)
                    corrections.append({
                        "row": row_index + 1,
                        "column": column_index + 1,
                        "old_value": raw_value,
                        "reason": reason,
                    })

        # Fill every gap using only colours that still have capacity below six.
        fill_number = 0
        for row_index in range(4):
            for column_index in range(3):
                if repaired[row_index][column_index] is not None:
                    continue

                candidates = [
                    colour for colour in VALID_COLOURS
                    if counts[colour] < MAXIMUM_PER_COLOUR
                ]
                if not candidates:
                    raise RuntimeError("No legal colour remains for scan repair")

                seed = (row_index + 1) * 101 + (column_index + 1) * 17 + fill_number
                chosen = candidates[pseudo_random_index(len(candidates), seed)]
                repaired[row_index][column_index] = chosen
                counts[chosen] += 1
                corrections[fill_number]["new_value"] = chosen
                fill_number += 1

        return repaired, corrections, counts

    # -----------------------------------------------------------------------
    # Convert scanned rows into three FIFO onboard-slot queues
    # -----------------------------------------------------------------------

    def create_slot_queues(repaired_array):
        return [
            [repaired_array[row][slot] for row in range(4)]
            for slot in range(3)
        ]

    # -----------------------------------------------------------------------
    # Dynamic-programming planner, including travel from and back to the star
    # -----------------------------------------------------------------------

    def route_cost(from_location, to_location):
        return abs(ROUTE_POSITION[to_location] - ROUTE_POSITION[from_location])

    def add_cost(first, second):
        return (
            first[0] + second[0],
            first[1] + second[1],
            first[2] + second[2],
        )

    def build_optimal_plan(slot_queues):
        """Minimise route travel, then colour visits, then slot changes."""
        memo = {}

        def solve(progress, current_location, previous_slot):
            state = (progress, current_location, previous_slot)
            if state in memo:
                return memo[state]

            if progress == (4, 4, 4):
                result = ((route_cost(current_location, STAR), 0, 0), [])
                memo[state] = result
                return result

            best_cost = None
            best_plan = None

            for slot_index in range(3):
                queue_index = progress[slot_index]
                if queue_index >= 4:
                    continue

                colour = slot_queues[slot_index][queue_index]
                travel = route_cost(current_location, colour)
                new_colour_visit = int(current_location != colour)
                slot_change = int(
                    previous_slot is not None and previous_slot != slot_index
                )

                new_progress = list(progress)
                new_progress[slot_index] += 1
                future_cost, future_plan = solve(
                    tuple(new_progress), colour, slot_index
                )

                total_cost = add_cost(
                    (travel, new_colour_visit, slot_change), future_cost
                )
                action = {
                    "colour": colour,
                    "slot": slot_index + 1,
                    "slot_position": queue_index + 1,
                }
                candidate_plan = [action] + future_plan

                if best_cost is None or total_cost < best_cost:
                    best_cost = total_cost
                    best_plan = candidate_plan

            memo[state] = (best_cost, best_plan)
            return memo[state]

        return solve((0, 0, 0), STAR, None)

    def attach_supply_positions(plan):
        """Assign each colour its mandatory R1/R2, then C1/C2/C3 order."""
        assigned_count = {colour: 0 for colour in VALID_COLOURS}
        for action in plan:
            colour = action["colour"]
            supply_index = assigned_count[colour]
            action["pickup_row"] = 1 if supply_index % 2 == 0 else 2
            action["pickup_column"] = supply_index // 2 + 1
            assigned_count[colour] += 1
        return plan

    # -----------------------------------------------------------------------
    # Safety and calibration
    # -----------------------------------------------------------------------

    def stop_all_motors():
        if RUNNING_ON_EV3:
            left_drive_motor.stop()
            right_drive_motor.stop()
            motor_c.stop()
            motor_d.stop()

    def require_all_calibration():
        values = {
            "MOTOR_C_SPEED": MOTOR_C_SPEED,
            "MOTOR_C_PICKUP_ROW_1": MOTOR_C_PICKUP_ROW_1,
            "MOTOR_C_PICKUP_ROW_2": MOTOR_C_PICKUP_ROW_2,
            "MOTOR_C_DEPOSIT_SLOT_1": MOTOR_C_DEPOSIT_SLOT_1,
            "MOTOR_C_DEPOSIT_SLOT_2": MOTOR_C_DEPOSIT_SLOT_2,
            "MOTOR_C_DEPOSIT_SLOT_3": MOTOR_C_DEPOSIT_SLOT_3,
            "MOTOR_D_NORMAL_SPEED": MOTOR_D_NORMAL_SPEED,
            "MOTOR_D_FLING_SPEED": MOTOR_D_FLING_SPEED,
            "MOTOR_D_OPEN_POSITION": MOTOR_D_OPEN_POSITION,
            "MOTOR_D_CHOMP_POSITION": MOTOR_D_CHOMP_POSITION,
            "MOTOR_D_FLING_POSITION": MOTOR_D_FLING_POSITION,
            "BLACK_THRESHOLD": BLACK_THRESHOLD,
            "LINE_BASE_SPEED": LINE_BASE_SPEED,
            "LINE_KP": LINE_KP,
            "JUNCTION_OVERSHOOT_MM": JUNCTION_OVERSHOOT_MM,
            "TURN_INTO_SEGMENT_ANGLE": TURN_INTO_SEGMENT_ANGLE,
        }
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError("GUYS PLS FILL: " + ", ".join(missing))

    # -----------------------------------------------------------------------
    # Motor C: common searchable supply-row and onboard-slot positions
    # -----------------------------------------------------------------------

    current_motor_c_position = [None]

    def move_motor_c(target_position, common_name):
        if current_motor_c_position[0] == common_name:
            return
        # DENZEL: Replace this run_target call if Motor C needs a custom motion.
        motor_c.run_target(
            MOTOR_C_SPEED, target_position, then=Stop.HOLD, wait=True
        )
        current_motor_c_position[0] = common_name

    def motor_c_pickup_row(row_number):
        if row_number == 1:
            move_motor_c(MOTOR_C_PICKUP_ROW_1, "MOTOR_C_PICKUP_ROW_1")
        else:
            move_motor_c(MOTOR_C_PICKUP_ROW_2, "MOTOR_C_PICKUP_ROW_2")

    def motor_c_deposit_slot(slot_number):
        positions = {
            1: (MOTOR_C_DEPOSIT_SLOT_1, "MOTOR_C_DEPOSIT_SLOT_1"),
            2: (MOTOR_C_DEPOSIT_SLOT_2, "MOTOR_C_DEPOSIT_SLOT_2"),
            3: (MOTOR_C_DEPOSIT_SLOT_3, "MOTOR_C_DEPOSIT_SLOT_3"),
        }
        target_position, common_name = positions[slot_number]
        move_motor_c(target_position, common_name)

    # -----------------------------------------------------------------------
    # Motor D: open, chomp, and fling
    # -----------------------------------------------------------------------

    def open_motor_d():
        # DENZEL: Replace if the chomper uses a stall/torque movement.
        motor_d.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_OPEN_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    def chomp_cube_with_motor_d():
        # DENZEL: Replace if the chomp should stop on torque instead of angle.
        motor_d.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_CHOMP_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    def fling_cube_with_motor_d():
        # DENZEL: Replace with the tested Motor D fling profile.
        motor_d.run_target(
            MOTOR_D_FLING_SPEED,
            MOTOR_D_FLING_POSITION,
            then=Stop.HOLD,
            wait=True,
        )
        motor_d.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_OPEN_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    # -----------------------------------------------------------------------
    # Map navigation movement hooks
    # -----------------------------------------------------------------------

    def line_trace_between_locations(from_location, to_location):
        """Line trace from the star/one segment to another segment.

        DENZEL: Replace this whole body with your tested two-sensor navigation.
        Use ROUTE_POSITION to determine direction and the number of junctions.
        A normal proportional loop is:
            error = S1.reflection() - S3.reflection()
            correction = LINE_KP * error
            Motor B speed = LINE_BASE_SPEED - correction
            Motor A speed = LINE_BASE_SPEED + correction
        Count a junction only once per new both-black event.
        """
        raise NotImplementedError(
            "DENZEL: implement line_trace_between_locations"
        )

    def retreat_from_segment_to_main_line(colour):
        """DENZEL: Reverse out of the colour segment and reacquire the line."""
        raise NotImplementedError(
            "DENZEL: implement retreat_from_segment_to_main_line"
        )

    def move_between_cube_columns(previous_column, target_column):
        """Move horizontally between C1, C2, and C3 in one colour group."""
        column_difference = target_column - previous_column
        distance_mm = column_difference * BRICK_SPACING_MM

        # DENZEL: Replace with your tested straight-drive movement. The required
        # distance is distance_mm. It will normally be 32 mm or 64 mm.
        raise NotImplementedError(
            "DENZEL: drive {} mm between cube columns".format(distance_mm)
        )

    def centre_robot_at_start_point():
        """Return to the star position; final heading is intentionally free."""
        # DENZEL: Replace with the final sensor-to-axle centring movement at star.
        raise NotImplementedError("DENZEL: centre robot at start point")

    # -----------------------------------------------------------------------
    # Debug display
    # -----------------------------------------------------------------------

    def display_plan(repaired_array, corrections, slot_queues, plan, cost):
        print("CORRECTED COLOUR ARRAY")
        for row in repaired_array:
            print(row)

        if corrections:
            print("SCAN CORRECTIONS")
            for correction in corrections:
                print(correction)

        print("SLOT QUEUES")
        for slot_index, queue in enumerate(slot_queues):
            print("Slot {}: {}".format(slot_index + 1, queue))

        print("OPTIMAL COLLECTION PLAN")
        for step, action in enumerate(plan):
            print("{:02d}. {:6s} R{}C{} -> Slot {}".format(
                step + 1,
                action["colour"],
                action["pickup_row"],
                action["pickup_column"],
                action["slot"],
            ))

        print("Round-trip route cost:", cost[0])
        print("Colour visits:", cost[1])

    # -----------------------------------------------------------------------
    # Run the complete algorithm
    # -----------------------------------------------------------------------

    try:
        repaired_array, corrections, final_counts = repair_colour_array(
            colour_array
        )
        slot_queues = create_slot_queues(repaired_array)
        total_cost, plan = build_optimal_plan(slot_queues)
        plan = attach_supply_positions(plan)
        display_plan(repaired_array, corrections, slot_queues, plan, total_cost)

        result = {
            "corrected_colour_array": repaired_array,
            "corrections": corrections,
            "colour_counts": final_counts,
            "slot_queues": slot_queues,
            "loading_plan": plan,
            "cost": total_cost,
            "ending_position": STAR,
            "slots_deposited": False,
        }

        if SIMULATION_MODE:
            return result

        if not RUNNING_ON_EV3:
            raise RuntimeError("Physical movement requires the EV3")

        # Validate every known value before the robot leaves the start point.
        require_all_calibration()
        open_motor_d()

        current_location = STAR
        current_column = None

        for action_index, action in enumerate(plan):
            colour = action["colour"]
            slot_number = action["slot"]

            if colour != current_location:
                if current_location != STAR:
                    retreat_from_segment_to_main_line(current_location)

                line_trace_between_locations(current_location, colour)
                current_location = colour

                # DENZEL: line tracing must finish aligned at Column 1.
                current_column = 1

            # Required physical supply order:
            # R1C1, R2C1, R1C2, R2C2, R1C3, R2C3.
            pickup_row = action["pickup_row"]
            pickup_column = action["pickup_column"]

            if current_column != pickup_column:
                move_between_cube_columns(current_column, pickup_column)
                current_column = pickup_column

            # Confirmed mechanism cycle:
            # C selects pickup row -> D chomps -> C selects slot -> D flings.
            motor_c_pickup_row(pickup_row)
            chomp_cube_with_motor_d()
            motor_c_deposit_slot(slot_number)
            fling_cube_with_motor_d()

            # Pre-position C for the next cube's supply row. This completes the
            # confirmed "C returns for next pickup" part of every cycle.
            if action_index + 1 < len(plan):
                next_pickup_row = plan[action_index + 1]["pickup_row"]
                motor_c_pickup_row(next_pickup_row)
            else:
                motor_c_pickup_row(1)

        # Leave all four cubes inside each slot. Do not deposit them here.
        retreat_from_segment_to_main_line(current_location)
        line_trace_between_locations(current_location, STAR)
        centre_robot_at_start_point()
        stop_all_motors()
        return result

    except Exception:
        stop_all_motors()
        raise


# Desktop/example test. The team's main file only needs to call sorting_algorithm().
if __name__ == "__main__":
    colour_array = [
        ["Blue", "Green", "Yellow"],
        ["White", "Blue", "Green"],
        ["Yellow", "Green", "Green"],
        ["Blue", "Yellow", "Yellow"],
    ]
    sorting_algorithm(colour_array)
