
"""State-machine navigation loop for an iRobot Create3 SDK robot.
 
Guides customers to items on a shopping list using BFS pathfinding,
path optimization, and reactive obstacle avoidance.
"""
 
import asyncio
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
 
robot = Create3(Bluetooth())
 
# ---------------------------------------------------------------------------
# Sensor distance thresholds
# ---------------------------------------------------------------------------
CLOSE_TH = 50   # "too close" — stop / turn
DIAG_TH  = 75   # diagonal sensor warning
TH       = 100  # general proximity threshold
 
# Direction indices into the IR proximity sensor array
LEFT          = 0
CLOSE_LEFT    = 1
DIAGONAL_LEFT = 2
FRONT         = 3
DIAGONAL_RIGHT = 4
CLOSE_RIGHT   = 5
RIGHT         = 6
 
# Driving speed (cm/s) used during free-move segments
SPEED = 15
 
# ---------------------------------------------------------------------------
# Occupancy grid  (1 = wall/obstacle, 0 = free)
# ---------------------------------------------------------------------------
uGrid = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]
 
# ---------------------------------------------------------------------------
# Store layout  {name: (grid_x, grid_y, width, height)}
# Only (grid_x, grid_y) is used for navigation; width/height are metadata.
# ---------------------------------------------------------------------------
DICTIONARY = {
    "Home":       (0,  0,  1, 1),
    "Veggies":    (1,  5,  1, 2),
    "Fruits":     (1,  11, 1, 2),
    "Meats":      (2,  15, 2, 1),
    "Pastries":   (2,  7,  1, 3),
    "Condiments": (5,  8,  1, 3),
    "Canned":     (6,  8,  1, 4),
    "Meals":      (7,  15, 2, 1),
    "Snacks":     (10, 8,  1, 4),
    "Cereal":     (11, 8,  1, 3),
    "Houseware":  (14, 8,  1, 3),
    "Dairy":      (13, 15, 2, 1),
    "Beverage":   (15, 10, 1, 3),
}
 
 
# ---------------------------------------------------------------------------
# Pathfinding helpers
# ---------------------------------------------------------------------------
 
def find_path(start: tuple, goal: tuple, grid: list) -> list:
    """BFS shortest path on a 2-D occupancy grid.
 
    Parameters
    ----------
    start : tuple[int, int]   Starting (x, y) coordinate.
    goal  : tuple[int, int]   Target  (x, y) coordinate.
    grid  : list[list[int]]   2-D grid; 0 = free, 1 = blocked.
 
    Returns
    -------
    list[tuple[int, int]]  Path from start to goal, or [] if unreachable.
    """
    if grid[start[1]][start[0]] != 0 or grid[goal[1]][goal[0]] != 0:
        return []
 
    queue   = [[start]]
    visited = {start}
 
    while queue:
        path   = queue.pop(0)
        x, y   = path[-1]
 
        if (x, y) == goal:
            return path
 
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
 
            if (0 <= nx < len(grid[0])) and (0 <= ny < len(grid)) and grid[ny][nx] == 0:
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append(path + [(nx, ny)])
 
    return []
 
 
def optimize(path: list) -> list:
    """Remove collinear mid-points to shorten a grid path.
 
    Parameters
    ----------
    path : list[tuple[int, int]]  Raw path from find_path().
 
    Returns
    -------
    list[tuple[int, int]]  Reduced path keeping only turns and endpoints.
    """
    if len(path) <= 2:
        return path
 
    result = [path[0]]
    for i in range(1, len(path) - 1):
        prev, curr, nxt = path[i - 1], path[i], path[i + 1]
        # Keep the point only if it represents a turn
        same_x = (prev[0] == curr[0] == nxt[0])
        same_y = (prev[1] == curr[1] == nxt[1])
        if not same_x and not same_y:
            result.append(curr)
 
    result.append(path[-1])
    return result
 
 
# ---------------------------------------------------------------------------
# Touch-button handler (blue light + flag)
# ---------------------------------------------------------------------------
pressed = 0
 
@event(robot.when_touched, [True, False])
async def touched(robot):
    global pressed
    await robot.set_lights_on_rgb(0, 0, 255)
    pressed = 1
 
 
# ---------------------------------------------------------------------------
# Main navigation coroutine
# ---------------------------------------------------------------------------
 
async def play(robot):
    """Run the shopping-guide navigation state machine.
 
    States
    ------
    AWAITING_INPUT      – Pop the next item from the queue.
    CALCULATING_PATH    – BFS + optimize from current position to target.
    CHECK_SENSORS       – Decide navigate vs. avoid.
    NAVIGATE_TO_NODE    – Drive to the next waypoint in short_path.
    CHECK_FINAL_POS     – Confirm arrival; go home if queue is empty.
    AVOID_OBSTACLE      – Reactive turn/drive to get around a blocker,
                          then recalculate path.
    """
    current_state = "AWAITING_INPUT"
 
    short_path  = []
    path_index  = 0
    current_item = None
 
    # Shopping list — replace or extend as needed
    queue = ["Veggies", "Fruits", "Meals"]
 
    while True:
        # Read IR sensors every loop tick (returns a list indexed by direction constants)
        ir      = await robot.get_ir_proximity()
        sensors = ir.sensors          # list[float]  — use sensors[FRONT] etc.
        await asyncio.sleep(0.05)
 
        # ------------------------------------------------------------------
        if current_state == "AWAITING_INPUT":
            if queue:
                current_item  = queue.pop(0)
                print(f"Next destination: {current_item}")
                current_state = "CALCULATING_PATH"
            else:
                # Nothing left — stay idle (or extend with user-input logic)
                print("Queue empty. Waiting.")
                await asyncio.sleep(1)
 
        # ------------------------------------------------------------------
        elif current_state == "CALCULATING_PATH":
            path_index = 0
            pos        = await robot.get_position()
            # round() is more accurate than int() for continuous coordinates
            current_pos = (round(pos.x), round(pos.y))
 
            target_xy   = DICTIONARY[current_item][:2]   # ← use only (x, y)
            og_path     = find_path(current_pos, target_xy, uGrid)
 
            if not og_path:
                print(f"No path found to {current_item}. Skipping.")
                current_state = "AWAITING_INPUT"
                continue
 
            short_path    = optimize(og_path)
            current_state = "CHECK_SENSORS"
 
        # ------------------------------------------------------------------
        elif current_state == "CHECK_SENSORS":
            front_blocked = sensors[FRONT]          > CLOSE_TH
            diag_blocked  = (sensors[DIAGONAL_LEFT] > DIAG_TH or
                             sensors[DIAGONAL_RIGHT] > DIAG_TH)
 
            if front_blocked or diag_blocked:
                current_state = "AVOID_OBSTACLE"
            else:
                current_state = "NAVIGATE_TO_NODE"
 
        # ------------------------------------------------------------------
        elif current_state == "NAVIGATE_TO_NODE":
            if path_index < len(short_path):
                target_x, target_y = short_path[path_index]
                await robot.navigate_to(target_x, target_y)
                path_index   += 1
                current_state = "CHECK_SENSORS"
            else:
                current_state = "CHECK_FINAL_POS"
 
        # ------------------------------------------------------------------
        elif current_state == "CHECK_FINAL_POS":
            pos = await robot.get_position()
            target_x, target_y = DICTIONARY[current_item][:2]
 
            if round(pos.x) == target_x and round(pos.y) == target_y:
                print(f"Arrived at {current_item}!")
            else:
                print(f"Warning: expected ({target_x},{target_y}), "
                      f"got ({round(pos.x)},{round(pos.y)}). Continuing anyway.")
 
            if not queue:
                # All items visited — navigate back home
                print("All items collected. Heading home.")
                pos         = await robot.get_position()
                current_pos = (round(pos.x), round(pos.y))
                home_path   = optimize(find_path(current_pos, DICTIONARY["Home"][:2], uGrid))
 
                for wx, wy in home_path:
                    await robot.navigate_to(wx, wy)
 
                print("Back home!")
            
            current_state = "AWAITING_INPUT"
 
        # ------------------------------------------------------------------
        elif current_state == "AVOID_OBSTACLE":
            """
            Simple wall-follow-style avoidance:
            1. If something is dead ahead, turn left.
            2. If the right side is now clear and the front is clear, hug
               the right wall by driving forward.
            3. If something is on the right too, turn right to find a gap.
            After any manoeuvre, recalculate the path so the robot re-plans
            from its new actual position rather than following a stale route.
            """
            if sensors[FRONT] > CLOSE_TH:
                # Obstacle straight ahead → turn left to find a gap
                await robot.turn_left(15)
 
            elif sensors[RIGHT] > CLOSE_TH:
                # Obstacle to the right while path is clear ahead — keep going
                await robot.set_wheel_speeds(SPEED, SPEED)
 
            elif sensors[LEFT] > CLOSE_TH:
                # Obstacle to the left but right is clear → turn right
                await robot.turn_right(15)
                await robot.set_wheel_speeds(SPEED, SPEED)
 
            else:
                # Space all around — nudge forward
                await robot.set_wheel_speeds(SPEED, SPEED)
 
            # Always recalculate after any avoidance manoeuvre so the path
            # reflects the robot's new physical position.
            current_state = "CALCULATING_PATH"
 
 
robot.play()
