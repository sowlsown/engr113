from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 30

def f2(value):
    return format(value, '.2f')

async def print_pos(robot):
    pos = await robot.get_position()
    print(f"Position: X={f2(pos.x)}, Y={f2(pos.y)}, Angle={f2(pos.heading)}")
    return pos

@event(robot.when_play)
async def play(robot):
    n_s = 0
    await print_pos(robot)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1
        elif n_s == 1:
            if sensors[5] > th:
                pos = await print_pos(robot)
                await robot.turn_left(180)
                await robot.move(pos.y)
                await print_pos(robot)

robot.play()