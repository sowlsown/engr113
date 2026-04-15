from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

speed = 30
th = 150
flag = 0

# globals to store positions
start_x = start_y = None
open1_x = open1_y = None
open2_x = open2_y = None
end_x = end_y = None

async def getpos(robot):
    return await robot.get_odometry()

@event(robot.when_touched, [True, False])
async def touched(robot):
    global flag
    await robot.set_lights_on_rgb(0, 0, 255)
    flag = 1

@event(robot.when_play)
async def play(robot):
    global speed, start_x, start_y, open1_x, open1_y, open2_x, open2_y, end_x, end_y

    # record starting position ONCE
    start_x, start_y, _ = await getpos(robot)

    # run the gap-finding sequence twice
    for i in range(2):

        n_s = 0
        speed = 30

        while True:
            sensors = (await robot.get_ir_proximity()).sensors

            # drive during search states
            if n_s in [0, 1, 2, 3]:
                await robot.set_wheel_speeds(speed, speed)
            else:
                await robot.set_wheel_speeds(0, 0)

            # 0: find first wall
            if n_s == 0:
                if sensors[3] > th:
                    await robot.turn_left(90)
                    n_s = 1

            # 1: search for first gap
            elif n_s == 1:
                if sensors[6] < th:
                    await robot.turn_right(90)
                    x, y, _ = await getpos(robot)
                    open1_x, open1_y = x, y
                    speed = 30
                    n_s = 2
                elif sensors[3] > th:
                    speed = -30

            # 2: find wall again
            elif n_s == 2:
                if sensors[3] > th:
                    await robot.turn_left(90)
                    speed = 30
                    n_s = 3

            # 3: search for second gap
            elif n_s == 3:
                if sensors[6] < th:
                    await robot.turn_right(90)
                    x, y, _ = await getpos(robot)
                    open2_x, open2_y = x, y
                    speed = 30
                    await robot.move(100)
                    x, y, _ = await getpos(robot)
                    end_x, end_y = x, y
                    break   # end this cycle, repeat loop
                elif sensors[3] > th:
                    speed = -30

    # after both cycles, wait for button
    while flag == 0:
        await robot.set_wheel_speeds(0, 0)

    # rerun path
    await robot.navigate(open1_x, open1_y)
    await robot.navigate(open2_x, open2_y)
    await robot.navigate(end_x, end_y)
    await robot.navigate(start_x, start_y)

    # done
    await robot.turn_left(360)
    await robot.set_wheel_speeds(0, 0)

robot.play()
