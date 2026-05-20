from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
import asyncio

robot = Create3(Bluetooth())
speed = 30
th = 30
move_task = None

def f2(value):
    return format(value, '.2f')

async def getpos(robot):
    '''Get the current position of the robot.'''
    pos = await robot.get_position()
    return (pos.x, pos.y)

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
async def on_play(robot):
    global move_task
    move_task = asyncio.ensure_future(robot.navigate_to(0, 1000))
    try:
        await move_task
    except asyncio.CancelledError:
        await robot.stop()
        await robot.turn_right(180)

@event(robot.when_play)
async def watch_ir(robot):
    global move_task
    while True:
        ir = await robot.get_ir_proximity()
        if ir is not None:
            # sensors[0]-[6] front-facing, higher value = closer obstacle
            # adjust threshold to your needs
            if any(s > 100 for s in ir.sensors[:6]):
                if move_task and not move_task.done():
                    move_task.cancel()
                break
        await asyncio.sleep(0.05)

robot.play()