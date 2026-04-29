from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Color, Create3
import asyncio

robot = Create3(Bluetooth())

speed = 30
th = 50

async def stop(robot):
    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_on_rgb(255, 0, 0)

async def forward(robot):
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, speed)

@event(robot.when_play)
async def play(robot):
    state = 0   # 0 = searching, 1 = follow left, 2 = follow right
    await forward(robot)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        left = sensors[0]
        right = sensors[6]

        # -------------------------
        # STATE 0: SEARCH FOR WALL
        # -------------------------
        if state == 0:

            if left > th:
                state = 1
            elif right > th:
                state = 2

        # -------------------------
        # STATE 1: FOLLOW LEFT WALL
        # -------------------------
        elif state == 1:
            await forward(robot)

            # lost wall
            if left < th:
                await robot.move(10)
                await robot.turn_left(90)
                await robot.move(25)


                sensors = (await robot.get_ir_proximity()).sensors
                if sensors[0] < th:
                    await stop(robot)
                    return
                else:
                    await forward(robot)

        # -------------------------
        # STATE 2: FOLLOW RIGHT WALL
        # -------------------------
        elif state == 2:
            await forward(robot)

            # lost wall
            if right < th:
                await robot.move(10)
                await robot.turn_right(90)
                await robot.move(25)

                sensors = (await robot.get_ir_proximity()).sensors
                if sensors[6] < th:
                    await stop(robot)
                    return
                else:
                    await forward(robot)

robot.play()
