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

# def right_obstacle(sensors):
#     '''Check if there is an obstacle to the right of the robot.'''
#     print(sensors[6])
#     return sensors[6] > th

@event(robot.when_play)
async def play(robot):
    n_s = 0
    flag = False
    await getpos(robot)
    while True:
        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await robot.set_wheel_speeds(speed, speed)
            if flag:
                n_s = 2
            else:
                n_s = 1
        elif n_s == 1:
            if not left_obstacle(sensors):
                await robot.set_wheel_speeds(0, 0)
                await robot.move(30)
                await robot.turn_left(90)
                flag = True
                n_s = 0
        elif n_s == 2:
            if not left_obstacle(sensors):
                await robot.set_wheel_speeds(0, 0)
                print("Reached the end of the wall.")
                return

robot.play()