from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 150

sGrid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0]
]

# def f2(value):
#     return format(value, '.2f')

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

def find_path(start, goal, grid, gx = 9, gy = 9):
    queue = [[start]]
    visited = set([start])
    
    while queue:
        path = queue.pop(0)
        x, y = path[-1]
        
        if (x, y) == goal:
            return path
            
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < gx and 0 <= ny < gy and grid[ny][nx] == 0:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    new_path = list(path)
                    new_path.append((nx, ny))
                    queue.append(new_path)
    return []

print(find_path((0, 0), (8, 8), sGrid))