# from irobot_edu_sdk.backend.bluetooth import Bluetooth
# from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
# from irobot_edu_sdk.music import Note

# robot = Create3(Bluetooth())

import doctest
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

def optimizePath(path: list):
    '''
    Reduce the path by removing unnecessary points. A point is unnecessary if it is in a straight line with the previous and next points.
    
    Parameters
    ----------
    path (list): list[tuple[int, int]] 
        The original path to be reduced.
    
    Returns
    -------
    list: list[tuple[int, int]]
        The reduced path.
        
    Examples
    --------
    A path with no reducible points is returned as-is:
    >>> reducePath([(0, 0), (1, 1)])
    [(0, 0), (1, 1)]

    A path where the middle point lies on a horizontal line is reduced:
    >>> reducePath([(0, 0), (1, 0), (2, 0)])
    [(0, 0), (2, 0)]

    A path where the middle point lies on a vertical line is reduced:
    >>> reducePath([(0, 0), (0, 1), (0, 2)])
    [(0, 0), (0, 2)]

    An L-shaped path keeps the corner point:
    >>> reducePath([(0, 0), (2, 0), (2, 3)])
    [(0, 0), (2, 0), (2, 3)]
    '''
    if len(path) <= 2:
        return path
    
    np = [path[0]]
    x, y = False, False
    for i in range(1, len(path)-1):
        prev = path[i-1]
        curr = path[i]
        next = path[i+1]
        x = not (prev[0] == curr[0] == next[0])
        y = not (prev[1] == curr[1] == next[1])
        if x and y:
            np.append(path[i])
        
    np.append(path[-1])
    return np
        
    
    

print(find_path((0, 0), (4, 8), sGrid))
print(optimizePath(find_path((0, 0), (4, 8), sGrid)))