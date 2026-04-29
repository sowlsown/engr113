from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 30

def f2(value):
    return format(value, '.2f')

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

def fl_obstacle(sensors):
    '''Check if there is an obstacle in front-left of the robot.'''
    print(sensors[1])
    return sensors[1] > th or sensors[2] > th

def fr_obstacle(sensors):
    '''Check if there is an obstacle in front-right of the robot.'''
    print(sensors[5])
    return sensors[5] > th or sensors[4] > th

@event(robot.when_play)
async def play(robot):
    n_s = 0
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1
        elif n_s == 1:
            if front_obstacle(sensors) or fl_obstacle(sensors) or fr_obstacle(sensors):
                await robot.set_wheel_speeds(0, 0)
                d = await getpos(robot)
                await robot.turn_left(180)
                n_s = 2
        elif n_s == 2:
            await robot.move(d.y)

robot.play()
