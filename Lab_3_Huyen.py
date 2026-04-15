#
# Licensed under 3-Clause BSD license available in the License file. Copyright (c) 2021-2022 iRobot Corporation. All rights reserved.
#

# Very basic example for avoiding front obstacles.

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 10
th = 150
flag = 0

## Rerun Press left button ## 
@event(robot.when_touched, [True, False])  # (.) button. 
async def touched(robot): 
    global flag 
    await robot.set_lights_on_rgb(0, 0, 255) 
    flag = 1


@event(robot.when_play)
async def play(robot):
    n_s = 0
    await robot.reset_navigation()  # get original pos
    
    while True:    
        sensors = (await robot.get_ir_proximity()).sensors

        if n_s == 0: #robot begins to move straight forward at speed assigned            
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1
        
        elif n_s == 1: #continues until meets wall, then turn right and  
            if sensors[3] > th:
                await robot.move(-5)
                await robot.turn_right(90)
                n_s = 2
                    
        elif n_s == 2: #moves straight along the wall until sensor can no longer see wall
            await robot.set_wheel_speeds(speed, speed)
            if sensors[0] < th: #can't sense wall anymore
                await robot.move(20)
                await robot.turn_left()
                firs_open = await robot.get_position()
                n_s = 3

        elif n_s == 3: # moves forward until next wall
            await robot.set_wheel_speeds(speed, speed)
            n_s = 4
        
        elif n_s == 4:
            if sensors[3] > th:
                await robot.move(-5)
                await robot.turn_left(90)
                n_s = 5
        
        elif n_s == 5:
            await robot.set_wheel_speeds(speed, speed)
            if sensors[6] < th: #can't sense wall anymore
                await robot.move(20)
                await robot.turn_right()
                sec_open = await robot.get_position() #position of second opening
                n_s = 6
        
        elif n_s == 6: 
            await robot.move(200 - sec_open.y)
            n_s = 7

        elif n_s == 7:
            if flag == 1:
                n_s = 8
        
        elif n_s == 8:
            # moves to first opening
            await robot.move(firs_open.y)
            await robot.turn_right(90)
            await robot.move(firs_open.x)
            await robot.turn_left(90)
            # moves to second opening
            await robot.move(sec_open.y - firs_open.y)
            await robot.turn_left(90)
            temper = await robot.get_position()
            await robot.move(temper.x - sec_open.x)
            await robot.turn_right(90)
            # move to the end
            await robot.move(200 - sec_open.y)

        elif n_s == 9:
            await robot.set_lights_on_rgb(255, 255, 0) 


robot.play()
