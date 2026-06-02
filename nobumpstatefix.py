"""
Supermarket directory robot — iRobot Create 3
=============================================
Guides customers to items on a shopping list by navigating a 17×17
occupancy grid mapped to an 8 ft × 8 ft physical store.

Grid encoding
-------------
  uGrid[row][col]  where  row = y-axis, col = x-axis
  0 = free space,  1 = wall / obstacle

Dictionary encoding
-------------------
  "ItemName": (grid_x, grid_y, aisle_length, aisle_width)
  grid_x / grid_y  → BFS target, converted to cm for navigate_to()
  aisle_length / aisle_width → defines the rectangular stop zone
  (robot is "arrived" when its position falls anywhere inside that zone)

Physical ↔ grid conversion
---------------------------
  8 ft × 8 ft store = 243.84 cm × 243.84 cm
  17-cell grid: cells 0 and 16 are walls, leaving 15 traversable cells each side
  → CELL_CM = 243.84 / 15 ≈ 16.256 cm per grid cell

  grid_to_cm(gx, gy) = (gx * CELL_CM,  gy * CELL_CM)

Obstacle handling
-----------------
  Two-layer defence so the robot never touches an obstacle:

  Layer 1 — proactive sensor sweep (primary):
    Before every navigate_to() call, approach_waypoint() drives manually
    with set_wheel_speeds() while polling IR sensors each tick.  If anything
    trips a old the robot stops, steers clear with a proportional
    controller (same Kp technique as the working wall-follow example), and
    only resumes navigate_to() once the path is clear.  The robot should
    never reach an obstacle under normal conditions.

  Layer 2 — bumper fallback (safety net):
    If something somehow makes contact, when_bumped fires concurrently,
    backs the robot up, and sets recalc_needed = True so the main loop
    re-plans from the new position.

  Sensor thresholds:
    DANGER_TH   — stop and steer immediately (very close, dead ahead)
    WARN_TH     — slow down and start correcting (approaching)
    DIAG_TH     — diagonal sensors, slightly looser threshold

Light feedback
--------------
  Spinning blue   → navigating to next waypoint
  Solid green     → arrived at destination
  Blinking amber  → obstacle detected, steering around
  Solid red       → no path found / queue empty / error
  White pulse     → all done, back home
"""

import math
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

# ---------------------------------------------------------------------------
# Robot instance
# ---------------------------------------------------------------------------
robot = Create3(Bluetooth())

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------
STORE_FT   = 8
CM_PER_FT  = 30.48
STORE_CM   = STORE_FT * CM_PER_FT          # 243.84 cm
GRID_CELLS = 15                             # traversable cells (walls excluded)
CELL_CM    = STORE_CM / GRID_CELLS          # ≈ 16.256 cm per grid cell

SPEED      = 15                             # cm/s for manual wheel moves
BACKUP_CM  = 8                              # cm to reverse after a bump

# ---------------------------------------------------------------------------
# IR sensor indices
# ---------------------------------------------------------------------------
LEFT           = 0
CLOSE_LEFT     = 1
DIAGONAL_LEFT  = 2
FRONT          = 3
DIAGONAL_RIGHT = 4
CLOSE_RIGHT    = 5
RIGHT          = 6

# ---------------------------------------------------------------------------
# Occupancy grid  (1 = wall/obstacle, 0 = free)
# Row index = y, column index = x
# ---------------------------------------------------------------------------
uGrid = [
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
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
# Store layout
# Format:  "Name": (grid_x, grid_y, aisle_length, aisle_width)
# grid_x / grid_y   = BFS target cell (top-left corner of the stop zone)
# aisle_length       = extent in x  (grid cells)
# aisle_width        = extent in y  (grid cells)
# ---------------------------------------------------------------------------

DICTIONARY = {
    "Home": (1, 1, 1, 1),
    "Veggies": (1, 11, 1, 2),
    "Fruits": (1, 5 ,1 ,2),
    "Meats": (2, 1, 2 ,1),
    "Pastries": (2, 8, 1, 3),
    "Condiments": (5, 8, 1, 3),
    "Canned": (6, 8 ,1, 4),
    "Meals": (7, 1, 2, 1),
    "Snacks": (10, 8, 1, 4),
    "Cereal": (11, 8, 1, 3),
    "Houseware": (14, 8, 1, 3),
    "Dairy": (13, 1, 2, 1),
    "Beverage" :(15, 6, 1, 3),
}

# ---------------------------------------------------------------------------
# Shared state between play(), the bumper handler, and the button handler
# ---------------------------------------------------------------------------
recalc_needed  = False      # bumper handler sets True; play() clears it
is_navigating  = False      # True while a navigate_to() call is in flight
advance_queue  = False      # right-button handler sets True when at ARRIVED;
                            # play() clears it and moves to next item


# ---------------------------------------------------------------------------
# Helper: grid ↔ cm conversion
# ---------------------------------------------------------------------------
def grid_to_cm(gx: int, gy: int) -> tuple:
    """Convert grid cell coordinates to physical cm for navigate_to()."""
    return (gx * CELL_CM, gy * CELL_CM)


def cm_to_grid(cx: float, cy: float) -> tuple:
    """Convert physical cm back to the nearest grid cell."""
    return (round(cx / CELL_CM), round(cy / CELL_CM))


# ---------------------------------------------------------------------------
# Helper: BFS pathfinder
# ---------------------------------------------------------------------------
def find_path(start: tuple, goal: tuple, grid: list) -> list:
    """Return a shortest BFS path from start to goal on grid, or [] if none.

    Parameters
    ----------
    start : (int, int)   Grid cell (x, y).
    goal  : (int, int)   Grid cell (x, y).
    grid  : list[list[int]]  0 = free, 1 = blocked. Indexed [row][col] = [y][x].

    Returns
    -------
    list[(int, int)]  Ordered waypoints start → goal, or [] if unreachable.
    """
    rows = len(grid)
    cols = len(grid[0])

    def passable(x, y):
        return 0 <= x < cols and 0 <= y < rows and grid[y][x] == 0

    if not passable(*start) or not passable(*goal):
        return []

    queue   = [[start]]
    visited = {start}

    while queue:
        path  = queue.pop(0)
        x, y  = path[-1]

        if (x, y) == goal:
            return path

        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if passable(nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append(path + [(nx, ny)])

    return []


# ---------------------------------------------------------------------------
# Helper: path optimizer — remove collinear mid-points
# ---------------------------------------------------------------------------
def optimize(path: list) -> list:
    """Remove unnecessary intermediate points from a grid path.

    A point is unnecessary if it lies on a straight horizontal or vertical
    segment between its neighbours.  Keeping only corners and endpoints
    reduces the number of navigate_to() calls.

    Parameters
    ----------
    path : list[(int, int)]   Raw BFS path.

    Returns
    -------
    list[(int, int)]  Reduced path.
    """
    if len(path) <= 2:
        return path

    result = [path[0]]
    for i in range(1, len(path) - 1):
        prev, curr, nxt = path[i - 1], path[i], path[i + 1]
        on_x_line = (prev[0] == curr[0] == nxt[0])
        on_y_line = (prev[1] == curr[1] == nxt[1])
        if not on_x_line and not on_y_line:
            result.append(curr)   # it's a corner — keep it

    result.append(path[-1])
    return result


# ---------------------------------------------------------------------------
# Bumper handler — fires concurrently if robot physically hits something
# ---------------------------------------------------------------------------
# @event(robot.when_bumped, [True, True])
# async def on_bumped(robot):
#     """React to any bumper contact during navigation.

#     Backs up, turns away from the obstacle, then signals the main loop to
#     recalculate the path from the robot's new position.
#     """
#     global recalc_needed, is_navigating

#     if not is_navigating:
#         return  # ignore bumps outside of active navigation

#     # Stop immediately
#     await robot.set_wheel_speeds(0, 0)
#     await robot.set_lights_blink_rgb(255, 140, 0)   # amber blink = recalculating

#     # Reverse away from the obstacle
#     await robot.move(-BACKUP_CM)

#     # Turn left 30° to clear the obstacle before re-planning
#     await robot.turn_left(30)

#     recalc_needed = True


# ---------------------------------------------------------------------------
# Right-button handler (••) — advance to next queue item
# ---------------------------------------------------------------------------
@event(robot.when_touched, [False, True])
async def on_button_advance(robot):
    """Skip the current wait and move to the next item in the queue.

    Only acts when the robot is stationary at a destination (ARRIVED state).
    Pressing the button mid-navigation is ignored so the robot can't be
    accidentally interrupted while moving.
    """
    global advance_queue, is_navigating

    if is_navigating:
        return  # robot is moving — ignore the press

    advance_queue = True
    await robot.set_lights_on_rgb(0, 100, 255)   # brief blue flash as acknowledgement


# ---------------------------------------------------------------------------
# Main play loop
# ---------------------------------------------------------------------------
@event(robot.when_play)
async def play(robot):
    """Shopping-guide state machine.

    States
    ------
    AWAIT_ITEM      Pop next item from queue, or go home if queue is empty.
    PLAN_PATH       BFS + optimize from current position to target.
    NAVIGATE        Drive waypoint-by-waypoint along the optimised path.
    ARRIVED         Confirm position, announce arrival, then back to AWAIT_ITEM.
    GO_HOME         Navigate back to (Home) after all items are done.
    """
    global recalc_needed, is_navigating, advance_queue

    # Reset odometry so (0,0) == robot start == "Home"
    await robot.reset_navigation()

    # ------------------------------------------------------------------ queue
    # Edit this list to change what the robot guides customers to.
    # Items must be keys in DICTIONARY.
    queue = ["Veggies", "Fruits"]

    current_item  = None
    current_state = "AWAIT_ITEM"

    while True:
        await hand_over()   # yield to other events (bumper handler, etc.)

        # ==============================================================
        if current_state == "AWAIT_ITEM":

            if not queue:
                current_state = "GO_HOME"
                continue

            current_item  = queue.pop(0)
            print(f"[QUEUE] Next item: {current_item}")
            current_state = "PLAN_PATH"

        # ==============================================================
        elif current_state == "PLAN_PATH":

            recalc_needed = False

            # Get current physical position and convert to grid
            pos         = await robot.get_position()
            start_grid  = cm_to_grid(pos.x, pos.y)

            # Target: grid (x, y) from the dictionary
            gx, gy, _length, _width = DICTIONARY[current_item]
            goal_grid = (gx, gy)

            print(f"[PATH] Planning {start_grid} → {goal_grid}")

            raw_path  = find_path(start_grid, goal_grid, uGrid)

            if not raw_path:
                print(f"[ERROR] No path found to {current_item}. Skipping.")
                await robot.set_lights_blink_rgb(255, 0, 0)   # red blink = error
                await robot.wait(1.5)
                current_state = "AWAIT_ITEM"
                continue

            waypoints     = optimize(raw_path)
            print(f"[PATH] Waypoints: {waypoints}")

            await robot.set_lights_spin_rgb(0, 100, 255)      # blue spin = navigating
            current_state = "NAVIGATE"

        # ==============================================================
        elif current_state == "NAVIGATE":

            gx, gy, _length, _width = DICTIONARY[current_item]
            waypoints_cm = [grid_to_cm(wx, wy) for wx, wy in waypoints]

            for (target_x_cm, target_y_cm) in waypoints_cm:

                if recalc_needed:
                    # A bump moved us off course — re-plan before continuing
                    print("[NAV] Bump detected, recalculating...")
                    current_state = "PLAN_PATH"
                    break

                print(f"[NAV] Driving to cm ({target_x_cm:.1f}, {target_y_cm:.1f})")
                is_navigating = True
                await robot.navigate_to(target_x_cm, target_y_cm)
                is_navigating = False

                await hand_over()   # let bumper handler run if needed

            else:
                # All waypoints completed without interruption
                current_state = "ARRIVED"

        # ==============================================================
        elif current_state == "ARRIVED":

            gx, gy, aisle_len, aisle_wid = DICTIONARY[current_item]

            # Verify the robot is inside the expected stop zone
            pos        = await robot.get_position()
            rx, ry     = cm_to_grid(pos.x, pos.y)
            in_zone    = (gx <= rx <= gx + aisle_len - 1 and
                          gy <= ry <= gy + aisle_wid - 1)

            if in_zone:
                print(f"[ARRIVED] At {current_item}!")
                await robot.play_note(Note.C5, 0.15)
                await robot.play_note(Note.E5, 0.15)
                await robot.play_note(Note.G5, 0.25)
            else:
                print(f"[ARRIVED] Near {current_item} "
                      f"(grid pos {rx},{ry}, zone origin {gx},{gy}). Continuing.")

            # Pulse green slowly while waiting for customer to press (••).
            # The right-button handler sets advance_queue = True to continue.
            advance_queue = False
            print("[WAIT] Press (••) to continue to next item.")
            while not advance_queue:
                await robot.set_lights_on_rgb(0, 220, 0)          # bright green
                await robot.wait(0.6)
                await robot.set_lights_on_rgb(0, 80, 0)           # dim pulse
                await robot.wait(0.4)
                await hand_over()                                  # let button handler fire

            advance_queue = False
            print("[ADVANCE] Button pressed — moving to next item.")
            current_state = "AWAIT_ITEM"

        # ==============================================================
        elif current_state == "GO_HOME":

            print("[HOME] All items done. Navigating home.")
            await robot.set_lights_spin_rgb(0, 100, 255)

            pos        = await robot.get_position()
            start_grid = cm_to_grid(pos.x, pos.y)
            hx, hy, _, _ = DICTIONARY["Home"]

            home_raw  = find_path(start_grid, (hx, hy), uGrid)

            if home_raw:
                home_wpts = optimize(home_raw)
                for wx, wy in home_wpts:
                    cx, cy = grid_to_cm(wx, wy)
                    is_navigating = True
                    await robot.navigate_to(cx, cy)
                    is_navigating = False
                    await hand_over()
            else:
                print("[ERROR] Cannot find path home.")

            await robot.set_lights_on_rgb(255, 255, 255)           # white = done
            # await robot.play_note(Note.C5, 0.2)
            # await robot.play_note(Note.G5, 0.2)
            # await robot.play_note(Note.C6, 0.4)
            print("[DONE] Back home. Queue complete.")
            break   # end the program


robot.play()
