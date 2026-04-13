from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 150
flag = 0

async def getpos(robot):
    return await robot.get_position()

## Rerun Press left button ##
@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    global flag
    await robot.set_lights_on_rgb(0, 0, 255)
    flag = 1


@event(robot.when_play)
async def play(robot):
    n_s = 0
    
    while True:
        sensors = (await robot.get_ir_proximity()).sensors

        print(n_s)

        # State 0: forward
        if n_s == 0:
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1

        # State 1: front obstacle?
        elif n_s == 1:
            if sensors[3] > th:
                await robot.move(-5)
                await robot.turn_left(90)
                n_s = 2

        # State 2: look for opening
        elif n_s == 2:
            await robot.set_wheel_speeds(speed, speed)
            if sensors[6] < th:
                await robot.move(30)
                await robot.turn_right(90)

                openingpos = await getpos(robot)
                await robot.move(50[PT1.1])
                endingpos = await getpos(robot)
                n_s = 3

        # State 3: return
        elif n_s == 3:
            if flag == 1:
                n_s = 4
        elif n_s == 4:
            await robot.turn_left(90)
            await robot.move(-openingpos.x) 
            await robot.turn_right(90)
            await robot.move(endingpos.y)

            n_s = 5
        elif n_s == 5:

            await robot.set_lights_on_rgb(255, 255, 0)


robot.play()
