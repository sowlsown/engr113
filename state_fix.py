"""State-machine navigation loop for an iRobot Edu SDK robot.

Uses proximity sensor thresholds to decide when to plan, navigate, or
avoid obstacles. External helpers expected in scope include
``find_path(dictionary)``, ``optimize(path)``, ``sensors(direction)``,
and a configured ``speed`` value.
"""

import asyncio
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

# Sensor distance thresholds (units depend on SDK configuration).
closeth = 50
diagth = 75
th = 100

# Direction indices for the sensors() helper.
LEFT = 0
CLOSE_LEFT = 1
DIAGONAL_LEFT = 2
FRONT = 3
DIAGONAL_RIGHT = 4
CLOSE_RIGHT = 5
RIGHT = 6

async def play(robot):
    """Run the navigation state machine.

    Parameters
    ----------
    robot : Robot or Create3
        Active robot instance used for navigation, motion, and sensing.

    Returns
    -------
    None
        Runs indefinitely until the coroutine is cancelled.

    Notes
    -----
    The coroutine loops forever, transitioning through these states:
    - ``AWAITING_INPUT``: idle placeholder before planning.
    - ``CALCULATING_PATH``: compute and optimize a path (optionally return
      to the origin before starting).
    - ``CHECK_SENSORS``: decide whether to avoid obstacles.
    - ``NAVIGATE_TO_NODE``: move along the optimized path.
    - ``AVOID_OBSTACLE``: simple reactive turning/driving.
    """
    current_state = "AWAITING_INPUT"

    start_pos = await robot.get_position()
    short_path = []
    path_index = 0

    # Main state machine loop.
    while True:
        await asyncio.sleep(0.1)

        if current_state == "AWAITING_INPUT":
            # Placeholder for user/system input before path planning.
            current_state = "CALCULATING_PATH"

        elif current_state == "CALCULATING_PATH":
            current_pos = await robot.get_position()

            # Build and optionally optimize the planned route.
            og_path = find_path(dictionary)

            if not og_path:
                current_state = "AWAITING_INPUT"
                continue

            if current_pos == (0, 0):
                short_path = optimize(og_path)
            else:
                await robot.navigate_to(0, 0, 90)
                short_path = optimize(og_path)

            current_state = "CHECK_SENSORS"

        elif current_state == "CHECK_SENSORS":
            # If obstacles are too close, switch to avoidance.
            if sensors(FRONT) > closeth or sensors(DIAGONAL_LEFT) > diagth or sensors(DIAGONAL_RIGHT) > diagth:
                current_state = "AVOID_OBSTACLE"
            else:
                current_state = "NAVIGATE_TO_NODE"

        elif current_state == "NAVIGATE_TO_NODE":
            if path_index < len(short_path):
                target_x, target_y = short_path[path_index]
                await robot.navigate_to(target_x, target_y)

                path_index += 1
                current_state = "CHECK_SENSORS"
            else:
                current_state = "CHECK_FINAL_POSITION"

        elif current_state == "CHECK_FINAL_POSITION":
            # Future hook for end-of-path handling.
            current_state = "AWAITING_INPUT"

        elif current_state == "AVOID_OBSTACLE":
            if sensors(FRONT) > closeth:
                await robot.turn_left(15)
            elif sensors(RIGHT) < closeth:
                 await robot.turn_right(10)
                 await robot.set_wheel_speeds(speed, speed)
            else:
                await robot.set_wheel_speeds(speed, speed)

robot.play()
