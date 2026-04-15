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
    startingpos = getpos(robot)
    n_s = 0
    
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        await robot.set_wheel_speeds(speed, speed)

        # finds wall
        if sensors[3] > th:
            await robot.turn_left(90)
            n_s == 1

        # searches for gap
        elif n_s == 1:
            await robot.set_wheel_speeds(speed, speed)
            if sensors[6] < th:
                await robot.turn_right(90)
                openingpos1 = getpos(robot)

                speed = 30
                n_s == 2
            elif sensors[3] > th:
                speed = -30

        # finds wall again
        elif n_s == 2:
            await robot.set_wheel_speeds(speed, speed)
            if sensors[3] > th:
                await robot.turn_left(90)
                n_s == 3

        # searches for gap again
        elif n_s == 3:
             await robot.set_wheel_speeds(speed, speed)
            if sensors[6] < th:
                await robot.turn_right(90)
                openingpos2 = getpos(robot)
                speed = 30

                await robot.move(100)
                endingpos = getpos(robot)

                n_s == 4
            elif sensors[3] > th:
                speed = -30

        # press button
        elif n_s == 4:
            if flag == 1:
                n_s == 5
        
        # rerun
        elif n_s == 5:
            await robot.turn_left(90)
            if openingpos1.x < startingpos.x:
                await robot.move(openingpos1.x)
            else:
                await robot.move(-openingpos1.x)
            await robot.turn_right(90)
            await robot.move(openingpos2.y)
            await robot.turn_left(90)
            if openingpos2.x < startingpos.x:
                await robot.move(openingpos2.x)
            else:
                await robot.move(-openingpos2.x)
            await robot.turn_right(90)
            await robot.move(endingpos.y)


        



            

        
            


       
