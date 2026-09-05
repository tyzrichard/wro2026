"""
WRO 2026 RoboMission Senior - drop-in sorting section.

TEAM MAIN CODE
    colour_array = scan_all_12_colours()
    move_robot_to_origin()
    sorting_algorithm(colour_array)
    # Robot is back at the origin point. Continue the team's hard-coded route.

The only public function needed by the team's main program is:
    sorting_algorithm(colour_array)

The function does NOT release the three completed slots. It starts at the blue
origin, collects all 12 cubes into the onboard FIFO slots, and returns to the origin.
The final heading is fixed. After returning to the origin point, the robot turns
to face away from all four brick-storage areas so the team's next hard-coded
movement always starts from the same pose.
"""


# DENZEL: This section assumes these corrected variable names already exist in
# the team's main file. It also allows this file to run in simulation on a PC.
try:
    motorA
    motorB
    motorC
    motorD
    left_sensor
    middle_sensor
    right_sensor
    acc
    wait
    Stop
    RUNNING_ON_EV3 = True
except NameError:
    # Allows algorithm simulation on a normal computer.
    RUNNING_ON_EV3 = False


def sorting_algorithm(colour_array):
    """Repair, plan, collect, sort, and return to the origin point.

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

    # Colour-change movement using the team's AccelerationController object.
    REVERSE_TO_BLACK_SPEED = None        # DENZEL: Safe reversing speed toward the black line.
    BLACK_LINE_POLL_MS = 10

    # A 180-degree arc changes the robot's lateral position by 2 * turn_radius.
    # The supplied medium example uses turn_radius=200. The small and large
    # values below assume equal spacing between all four colour zones.
    ARC_RADIUS_SMALL = 100               # DENZEL: Confirm adjacent-zone radius.
    ARC_RADIUS_MEDIUM = 200              # DENZEL: Given alternate-zone radius.
    ARC_RADIUS_LARGE = 300               # DENZEL: Confirm end-to-end radius.

    # Relative route positions are used only to choose the fastest colour order.
    # The origin point is between green and blue in the supplied drawing.
    ORIGIN = "Origin"
    FINAL_HEADING = "Away from brick storage areas"
    ROUTE_POSITION = {
        "White": -3,                    # DENZEL: Adjust if measured route differs.
        "Green": -1,                    # DENZEL: Adjust if measured route differs.
        ORIGIN: 0,
        "Blue": 1,                      # DENZEL: Adjust if measured route differs.
        "Yellow": 3,                    # DENZEL: Adjust if measured route differs.
    }

    VALID_COLOURS = ("White", "Green", "Blue", "Yellow")

    # Explicit directed transitions. According to turn_degrees():
    #   +180 = right/downward arc on the supplied map
    #   -180 = left/upward arc on the supplied map
    # This table covers every possible change between two different colours.
    COLOUR_CHANGE_ARCS = {
        # Small blue arcs: neighbouring colour zones.
        ("White", "Green"): (+180, ARC_RADIUS_SMALL),
        ("Green", "White"): (-180, ARC_RADIUS_SMALL),
        ("Green", "Blue"): (+180, ARC_RADIUS_SMALL),
        ("Blue", "Green"): (-180, ARC_RADIUS_SMALL),
        ("Blue", "Yellow"): (+180, ARC_RADIUS_SMALL),
        ("Yellow", "Blue"): (-180, ARC_RADIUS_SMALL),

        # Medium orange arcs: skip one colour zone.
        ("White", "Blue"): (+180, ARC_RADIUS_MEDIUM),
        ("Blue", "White"): (-180, ARC_RADIUS_MEDIUM),
        ("Green", "Yellow"): (+180, ARC_RADIUS_MEDIUM),
        ("Yellow", "Green"): (-180, ARC_RADIUS_MEDIUM),

        # Large brown arcs: one end of the storage area to the other.
        ("White", "Yellow"): (+180, ARC_RADIUS_LARGE),
        ("Yellow", "White"): (-180, ARC_RADIUS_LARGE),
    }

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
                result = ((route_cost(current_location, ORIGIN), 0, 0), [])
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

        return solve((0, 0, 0), ORIGIN, None)

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
            motorB.stop()
            motorA.stop()
            motorC.stop()
            motorD.stop()

    def hold_final_pose():
        """Hold the fixed original point pose after successful sorting."""
        if RUNNING_ON_EV3:
            motorB.hold()
            motorA.hold()
            motorC.stop()
            motorD.stop()

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
            "REVERSE_TO_BLACK_SPEED": REVERSE_TO_BLACK_SPEED,
        }
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError("DENZEL must fill: " + ", ".join(missing))

    # -----------------------------------------------------------------------
    # Motor C: common searchable supply-row and onboard-slot positions
    # -----------------------------------------------------------------------

    current_motorC_position = [None]

    def move_motorC(target_position, common_name):
        if current_motorC_position[0] == common_name:
            return
        # DENZEL: Replace this run_target call if Motor C needs a custom motion.
        motorC.run_target(
            MOTOR_C_SPEED, target_position, then=Stop.HOLD, wait=True
        )
        current_motorC_position[0] = common_name

    def motorC_pickup_row(row_number):
        if row_number == 1:
            move_motorC(MOTOR_C_PICKUP_ROW_1, "MOTOR_C_PICKUP_ROW_1")
        else:
            move_motorC(MOTOR_C_PICKUP_ROW_2, "MOTOR_C_PICKUP_ROW_2")

    def motorC_deposit_slot(slot_number):
        positions = {
            1: (MOTOR_C_DEPOSIT_SLOT_1, "MOTOR_C_DEPOSIT_SLOT_1"),
            2: (MOTOR_C_DEPOSIT_SLOT_2, "MOTOR_C_DEPOSIT_SLOT_2"),
            3: (MOTOR_C_DEPOSIT_SLOT_3, "MOTOR_C_DEPOSIT_SLOT_3"),
        }
        target_position, common_name = positions[slot_number]
        move_motorC(target_position, common_name)

    # -----------------------------------------------------------------------
    # Motor D: open, chomp, and fling
    # -----------------------------------------------------------------------

    def open_motorD():
        # DENZEL: Replace if the chomper uses a stall/torque movement.
        motorD.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_OPEN_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    def chomp_cube_with_motorD():
        # DENZEL: Replace if the chomp should stop on torque instead of angle.
        motorD.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_CHOMP_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    def fling_cube_with_motorD():
        # DENZEL: Replace with the tested Motor D fling profile.
        motorD.run_target(
            MOTOR_D_FLING_SPEED,
            MOTOR_D_FLING_POSITION,
            then=Stop.HOLD,
            wait=True,
        )
        motorD.run_target(
            MOTOR_D_NORMAL_SPEED,
            MOTOR_D_OPEN_POSITION,
            then=Stop.HOLD,
            wait=True,
        )

    # -----------------------------------------------------------------------
    # Map navigation movement hooks
    # -----------------------------------------------------------------------

    def line_trace_between_locations(from_location, to_location):
        """Handle only Origin-to-colour or colour-to-Origin movement.

        Direct colour-to-colour movement is handled by
        move_between_colour_segments() using the new semicircular arc route.
        """
        # DENZEL: Insert your existing movement between the origin point and a
        # colour zone. This is intentionally separate from colour-change arcs.
        raise NotImplementedError(
            "DENZEL: implement origin route from {} to {}".format(
                from_location, to_location
            )
        )

    def black_line_detected_while_reversing():
        """Return True when the chosen sensor detects the main black line."""
        # DENZEL: Replace this body with the sensor expression already used in
        # your main program. Keep the function name so no other code changes.
        # Example only, if your sensor mode returns a 0-to-255 light value:
        # return middle_sensor.read("DC")[0] <= BLACK_THRESHOLD
        raise NotImplementedError(
            "DENZEL: implement BLACK_LINE_DETECTED_WHILE_REVERSING"
        )

    def reverse_until_black_line():
        """Reverse from a storage segment until the main black line is found."""
        while not black_line_detected_while_reversing():
            motorA.run(-REVERSE_TO_BLACK_SPEED)
            motorB.run(-REVERSE_TO_BLACK_SPEED)
            wait(BLACK_LINE_POLL_MS)

        motorA.stop()
        motorB.stop()

    def retreat_from_segment_to_main_line(colour):
        """Reverse to the main line and square the robot against that line."""
        reverse_until_black_line()

        # The team's fool-proof self-alignment routine.
        acc.line_following_blackvar()

    def move_between_colour_segments(from_colour, to_colour):
        """Move directly between any two storage areas using a 180-degree arc.

        Sequence:
          1. Reverse until a sensor detects the main black line.
          2. Self-align at the black line.
          3. Spot-turn 180 degrees.
          4. Perform the required small, medium, or large 180-degree arc.
          5. Self-align at the destination colour line.
        """
        if from_colour == to_colour:
            return

        transition = COLOUR_CHANGE_ARCS.get((from_colour, to_colour))
        if transition is None:
            raise ValueError(
                "Unsupported colour transition: {} to {}".format(
                    from_colour, to_colour
                )
            )

        arc_angle, arc_radius = transition

        # Finished collecting from the current colour storage area.
        retreat_from_segment_to_main_line(from_colour)

        # Face away from the blocks before starting the rainbow-shaped arc.
        acc.turn_degrees(180, mode="spot")

        # Arrive facing into the next colour storage area.
        acc.turn_degrees(
            arc_angle,
            mode="arc",
            turn_radius=arc_radius,
        )

        # Correct accumulated turning error at the destination line.
        acc.line_following_blackvar()

    def move_between_cube_columns(previous_column, target_column):
        """Move horizontally between C1, C2, and C3 in one colour group."""
        column_difference = target_column - previous_column
        distance_mm = column_difference * BRICK_SPACING_MM

        # DENZEL: Replace with your tested straight-drive movement. The required
        # distance is distance_mm. It will normally be 32 mm or 64 mm.

        # acc.move_distance(32)
        
        raise NotImplementedError(
            "DENZEL: drive {} mm between cube columns".format(distance_mm)
        )

    def centre_robot_at_origin():
        """Centre the robot on the origin point before its final turn."""
        # DENZEL: Replace with the final sensor-to-axle centring movement at origin.
        raise NotImplementedError("DENZEL: centre robot at origin point")

    def turn_left_90_at_origin():
        """Make the tested 90-degree left point turn at the origin point."""
        # DENZEL: Replace this body with the team's tested left 90-degree turn.
        # Common replacement term: ORIGIN_TURN_LEFT_90
        # Keep the centre of the robot on the origin point during the point turn.

        # acc.turn_degrees(-90, mode="spot")

        raise NotImplementedError("DENZEL: implement ORIGIN_TURN_LEFT_90")

    def turn_right_90_at_origin():
        """Make the tested 90-degree right point turn at the origin point."""
        # DENZEL: Replace this body with the team's tested right 90-degree turn.
        # Common replacement term: ORIGIN_TURN_RIGHT_90
        # Keep the centre of the robot on the origin point during the point turn.
        raise NotImplementedError("DENZEL: implement ORIGIN_TURN_RIGHT_90")

    def square_robot_facing_away_from_storage():
        """Correct small turn errors while already facing away from storage."""
        # DENZEL: Use Sensors 1 and 3 to square the robot on the departure line.
        # Common replacement term: ORIGIN_FINAL_SQUARE_AWAY_FROM_STORAGE
        # Pseudocode:
        #   while Sensor 1 and Sensor 3 are not equally aligned to the line:
        #       move only the side that has not reached the line
        #   stop with the front of the robot pointing toward the open field
        # Do not turn 180 degrees here; this helper only corrects a small angle.
        raise NotImplementedError(
            "DENZEL: implement ORIGIN_FINAL_SQUARE_AWAY_FROM_STORAGE"
        )

    def face_away_from_storage_at_origin(last_colour):
        """Finish in one fixed direction regardless of the collection route.

        ROUTE_POSITION values below the origin point represent White/Green; the robot
        arrives from that side facing toward increasing route positions. Values
        above the origin point represent Blue/Yellow; it arrives facing toward decreasing
        route positions. The opposite 90-degree turns therefore point both cases
        toward the same open-field direction, away from the brick storage.

        DENZEL: This assumes line_trace_between_locations(last_colour, ORIGIN)
        reaches the origin point facing in its direction of travel. If your line-tracing
        function reverses into the origin, swap the left and right calls below.
        """
        if ROUTE_POSITION[last_colour] < ROUTE_POSITION[ORIGIN]:
            # Returning from White/Green: turn left toward the open field.
            turn_left_90_at_origin()
        else:
            # Returning from Blue/Yellow: turn right toward the open field.
            turn_right_90_at_origin()

        square_robot_facing_away_from_storage()

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
            "ending_position": ORIGIN,
            "ending_heading": FINAL_HEADING,
            "slots_deposited": False,
        }

        if SIMULATION_MODE:
            return result

        if not RUNNING_ON_EV3:
            raise RuntimeError("Physical movement requires the EV3")

        # Validate every known value before the robot leaves the origin point.
        require_all_calibration()
        open_motorD()

        current_location = ORIGIN
        current_column = None

        for action_index, action in enumerate(plan):
            colour = action["colour"]
            slot_number = action["slot"]

            if colour != current_location:
                if current_location == ORIGIN:
                    # First colour: use the team's Origin-to-colour movement.
                    line_trace_between_locations(ORIGIN, colour)
                else:
                    # Later colours: reverse, align, spot-turn, arc, and align.
                    move_between_colour_segments(current_location, colour)

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
            motorC_pickup_row(pickup_row)
            chomp_cube_with_motorD()
            motorC_deposit_slot(slot_number)
            fling_cube_with_motorD()

            # Pre-position C for the next cube's supply row. This completes the
            # confirmed "C returns for next pickup" part of every cycle.
            if action_index + 1 < len(plan):
                next_pickup_row = plan[action_index + 1]["pickup_row"]
                motorC_pickup_row(next_pickup_row)
            else:
                motorC_pickup_row(1)

        # Leave all four cubes inside each slot. Do not deposit them here.
        retreat_from_segment_to_main_line(current_location)
        line_trace_between_locations(current_location, ORIGIN)
        centre_robot_at_origin()
        face_away_from_storage_at_origin(current_location)
        hold_final_pose()
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
