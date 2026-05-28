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

@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    global pressed
    await robot.set_lights_on_rgb(0, 0, 255)
    pressed = 1

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

    #"key":(x,y, length, height)
    dictionary = {
        "Home": (0,0,1, 1),
        "Veggies":(1,5,1,2),
        "Fruits":(1,11,1,2),
        "Meats":(2,15,2,1),
        "Pastries":(2,7,1,3),
        "Condiments": (5,8,1,3),
        "Canned":(6,8,1,4),
        "Meals":(7,15,2,1),
        "Snacks":(10,8,1,4),
        "Cereal":(11,8,1,3),
        "Houseware":(14,8,1,3),
        "Dairy":(13,15,2,1),
        "Beverage":(15,10,1,3),
    }

    queue = [
        "Veggies", "Fruits", "Meals"
    ]

    # Main state machine loop.
    while True:
        await asyncio.sleep(0.1)

        if current_state == "AWAITING_INPUT":


            if len(queue) != None:
                item = queue.pop(0)
                current_state = "CALCULATING_PATH"

            else:
                print("Check the list!")


        elif current_state == "CALCULATING_PATH":
            current_pos = await robot.get_position()

            # Build and optionally optimize the planned route.
            og_path = find_path(dictionary[item])

            if not og_path:
                current_state = "AWAITING_INPUT"
                continue

            if current_pos == ():
                short_path = optimize(og_path)
            else:
                await robot.navigate_to()
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

        elif current_state == "GO_HOME":
            if len(queue) == 0:

                current_state = "AWAITING_INPUT"
            else:


robot.play()
