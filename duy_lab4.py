from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

speed = 30
th = 150

async def stop(robot):
    await robot.set_wheel_speeds(0, 0)

async def forward(robot):
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, speed)

@event(robot.when_play)
async def play(robot):
    await forward(robot)

    state = 0   
    # 0 = searching, 1 = follow left, 2 = follow right

    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        left = sensors[0]
        right = sensors[6]


        if state == 0:
            await forward(robot)

            if left > th:
                state = 1   # follow left wall
            elif right > th:
                state = 2   # follow right wall


        elif state == 1:
            await forward(robot)

            # lost the wall -> stop sequence
            if left < th:
                await robot.move(40)
                await robot.turn_left(90)
                await forward(robot)




        elif state == 2:
            await forward(robot)

            # lost the wall -> stop sequence
            if right < th:
                await robot.move(40)
                await robot.turn_right(90)
                await forward(robot)

