from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 150
flag = 0

oPos = []

async def getpos(robot):
    '''Get the current position of the robot.'''
    return await robot.get_position()

def front_obstacle(sensors):
    '''Check if there is an obstacle in front of the robot.'''
    print(sensors[3])
    return sensors[3] > th

def left_obstacle(sensors):
    '''Check if there is an obstacle to the left of the robot.'''
    print(sensors[0])
    return sensors[0] > th

def right_obstacle(sensors):
    '''Check if there is an obstacle to the right of the robot.'''
    print(sensors[6])
    return sensors[6] > th

# async def determineDirection(r, s):
#     '''Turn a certain direction based on the sensor readings.'''
#     if front_obstacle(s) and left_obstacle(s):
#         await r.turn_right(90)
#         return 'front and left detected: turn right'
#     elif front_obstacle(s) and right_obstacle(s):
#         await r.turn_left(90)
#         return 'front and right detected: turn left'
#     elif front_obstacle(s):
#         await r.turn_left(90)
#         return 'only front detected: default to left turn'
    
async def hitWall(r, s):
    if front_obstacle(s):
        await r.set_wheel_speeds(0, 0)
        await r.move(-10)
        await robot.turn_right(180)
        
    pos = await getpos(r)
    return pos.header

@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    '''Handle the event when the robot is touched.'''
    global flag
    await robot.set_lights_on_rgb(0, 255, 0)
    flag = 1


@event(robot.when_play)
async def play(robot):
    global oPos, flag
    n_s = 0
    switch = False
    
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            print("state 0")
            await robot.set_lights_on_rgb(0, 0, 255)
            await robot.set_wheel_speeds(speed, speed)
        elif n_s == 1:
            print("state 1")
            if front_obstacle(sensors):
                await robot.set_wheel_speeds(0, 0)
                await robot.move(-5)
                oPos[-1]['y'] = (await getpos(robot)).y
                await robot.turn_right(90)
                switch = not switch
                n_s = 0
        elif n_s == 2:
            print("state 2")
            angle = await hitWall(robot, sensors) # returns
            if 90 < angle < 270:
                if not left_obstacle(sensors):
                    await robot.set_wheel_speeds(0, 0)
                    await robot.move(30) # nudge to clear opening
                    oPos[-1]['x'] = (await getpos(robot)).x
                    await robot.turn_left(90)
                    switch = not switch
                    n_s = 0
            else:
                if not right_obstacle(sensors):
                    await robot.set_wheel_speeds(0, 0)
                    await robot.move(30) # nudge to clear opening
                    oPos[-1]['x'] = (await getpos(robot)).x
                    await robot.turn_right(90)
                    switch = not switch
                    n_s = 0
        elif n_s == 3:
            print("state 3")
            await robot.set_lights_on_rgb(255, 0, 0)
            await robot.set_wheel_speeds(0, 0)
            if flag == 1:
                await robot.reset_navigation()
                n_s = 4
            
        elif n_s == 4:
            print("state 4")
            await robot.navigate_to(oPos[0]['x'], oPos[0]['y'], 90)
            await robot.navigate_to(oPos[1]['x'], oPos[1]['y'], 90)
            await robot.move(200 - oPos[1]['y'])
            fpos = await getpos(robot)
            print(f"Final position: ({fpos.x}, {fpos.y})")
            print("end")

robot.play()
