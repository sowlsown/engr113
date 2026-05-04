from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Create3
import math

robot = Create3(Bluetooth())
speed = 25
th = 150

bread = '1'

category = {bread: }

# 1. The Grid Map (0 = empty, 1 = wall)
grid_map = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0], # Added some walls to test it!
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0]

]

# 2. The Pathfinding Algorithm
def find_path(start, goal, grid):
    queue = [[start]]
    visited = set([start])
    
    while queue:
        path = queue.pop(0)
        x, y = path[-1]
        
        if (x, y) == goal:
            return path
            
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < 8 and 0 <= ny < 8 and grid[ny][nx] == 0:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    new_path = list(path)
                    new_path.append((nx, ny))
                    queue.append(new_path)
    return []

# 3. Test and Visualize
start_pos = (0, 0)
goal_pos = (4, 7)
path = find_path(start_pos, goal_pos, grid_map)

print(f"Start: {start_pos} | Goal: {goal_pos}")
print(f"Path Coordinates: {path}\n")

# Draw the map in the console
print("Map Visualization:")
for y in range(8):
    row = ""
    for x in range(8):
        if (x, y) == start_pos:
            row += " S " # Start
        elif (x, y) == goal_pos:
            row += " G " # Goal
        elif (x, y) in path:
            row += " * " # The Path
        elif grid_map[y][x] == 1:
            row += "[X]" # Wall
        else:
            row += " . " # Empty space
    print(row)
    
