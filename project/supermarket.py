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
    navigate_path() keeps the full planned path and drives each leg with
    navigate_to() running as a background asyncio task, while a concurrent
    loop polls the IR sensors every POLL_S seconds.  The robot moves
    unhindered until a sensor trips a threshold; then the navigate_to() task
    is cancelled and sidestep_obstacle() runs a wall-aware doorway-style
    side-step:
      * decisions use the MEDIAN of several IR reads (read_ir_median), since
        a single sample is noisy;
      * it only steps toward a side with no wall (side_blocked uses the outer
        + close sensors), and the FRONT sensor watches the step path for a
        wall while sliding;
      * how far to slide / advance is computed from the IR reading itself
        (clearance_distance, via the ir_to_cm log curve);
      * it re-checks the sensors and confirms the obstacle is gone before
        nudging forward past it.
    After the detour the robot rejoins the planned route at its closest
    remaining waypoint (closest_waypoint).

  Layer 1b — barrier marking when boxed in:
    If there is no room to get around the obstacle, its grid cell is flipped
    to a wall (mark_barrier) and the path is re-planned so BFS routes around
    it.  These marks are temporary: reset_grid() restores the pristine layout
    (DEFAULT_GRID) at the start of every new item so they don't leak forward.

  Layer 2 — bumper fallback (safety net):
    If something somehow makes contact, when_bumped fires concurrently,
    backs the robot up, and sets recalc_needed = True so the main loop
    re-plans from the new position.

  Sensor thresholds:
    WARN_TH     — front sensor: trip a side-step when an object is ahead
    DIAG_TH     — diagonal sensors, slightly looser threshold
    WALL_TH     — side / step-path sensors: counts as a wall to avoid

Light feedback
--------------
  Spinning blue   → navigating to next waypoint
  Solid green     → arrived at destination
  Blinking amber  → obstacle detected, steering around
  Solid red       → no path found / queue empty / error
  White pulse     → all done, back home
"""

import asyncio
import copy
import math
import statistics
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

SPEED      = 15                             # cm/s cruise speed
BACKUP_CM  = 8                              # cm to reverse after a bump

# Sensor-aware navigation tuning.  navigate_to() drives the full leg at speed
# while a concurrent asyncio task polls the IR sensors every POLL_S seconds.
# The drive is only interrupted if an obstacle is actually detected, so the
# robot moves unhindered the rest of the time.
POLL_S        = 0.5   # seconds between obstacle checks during a drive
ARRIVE_TOL_CM = 5     # within this many cm of a waypoint counts as "reached"
MAX_SIDESTEPS = 6     # give up and re-plan after this many consecutive side-steps

# Obstacle-avoidance geometry.
ROBOT_CLEARANCE_CM = 18    # ~robot radius + a little; how far sideways to clear its body
SAFETY_MARGIN_CM   = 6     # extra buffer added to every clearance move
SENSOR_RANGE_CM    = 25    # max usable IR distance (also the value used when reading <= 0)
SIDE_STEP_CM       = 5     # incremental lateral hop while checking for walls mid-step
MAX_SIDE_STEP_CM   = 45    # never slide further than this hunting for room

# Sensors are noisy, so every avoidance *decision* is made on the median of
# several quick reads rather than a single sample.
IR_SAMPLES      = 3   # IR snapshots averaged (median) per decision
IR_SAMPLE_GAP_S = 0.02  # delay between those snapshots

# Sensor thresholds — tune these to your physical environment.
# Higher raw IR value = closer object.
WARN_TH    = 55     # front sensor: object ahead → trigger a side-step
DIAG_TH    = 60     # diagonal sensor warning threshold
WALL_TH    = 50     # side / step-path sensor level that counts as a wall

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

# Pristine copy of the store layout.  During an item the robot may mark
# temporary barriers into uGrid where it finds an impassable obstacle; after
# each item the grid is restored from this default so those marks don't leak
# into the next trip.
DEFAULT_GRID = copy.deepcopy(uGrid)

# ---------------------------------------------------------------------------
# Store layout
# Format:  "Name": (grid_x, grid_y, aisle_length, aisle_width)
# grid_x / grid_y   = BFS target cell (top-left corner of the stop zone)
# aisle_length       = extent in x  (grid cells)
# aisle_width        = extent in y  (grid cells)
# ---------------------------------------------------------------------------
DICTIONARY = {
    "Home":       (1,  1,  1, 1),   # robot starting position
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
        path = queue.pop(0)
        x, y = path[-1]

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
# Helper: is something blocking the path dead/diagonally ahead?
# ---------------------------------------------------------------------------
def obstacle_ahead(sensors) -> bool:
    """True if the front or either diagonal sensor is over its threshold."""
    return (sensors[FRONT]          >= WARN_TH or
            sensors[DIAGONAL_LEFT]  >= DIAG_TH or
            sensors[DIAGONAL_RIGHT] >= DIAG_TH)


# ---------------------------------------------------------------------------
# Helper: median IR read (IR sensors are noisy — sample a few and take median)
# ---------------------------------------------------------------------------
async def read_ir_median(robot, samples: int = IR_SAMPLES) -> list:
    """Return a per-channel median of `samples` quick IR proximity reads.

    A single IR snapshot is noisy and prone to spikes/dropouts; taking the
    median of a few rejects one-off outliers so avoidance decisions aren't
    made on a bad reading.
    """
    rows = []
    for _ in range(samples):
        rows.append((await robot.get_ir_proximity()).sensors)
        await asyncio.sleep(IR_SAMPLE_GAP_S)
    return [statistics.median([r[i] for r in rows]) for i in range(len(rows[0]))]


# ---------------------------------------------------------------------------
# Helper: convert a raw IR reading to an estimated distance / a move distance
# ---------------------------------------------------------------------------
def ir_to_cm(sensor_value: float) -> float:
    """Estimate the distance (cm) to an object from a raw IR reading.

    Same log curve as the working doorway demo (conv_to_cm), clamped to a
    sane [0, SENSOR_RANGE_CM] band so noisy extremes can't produce silly
    distances.  Higher reading → closer object → smaller distance.
    """
    if sensor_value <= 0:
        return SENSOR_RANGE_CM
    cm = math.log(sensor_value / 5296) / (-0.348)
    return max(0.0, min(SENSOR_RANGE_CM, cm))


def clearance_distance(sensor_value: float) -> float:
    """How far the robot should move to clear an object at this IR reading.

    Always covers the robot's body (ROBOT_CLEARANCE_CM) plus a safety buffer,
    and adds more the *closer* the object is: a near object (small estimated
    gap) needs a bigger move to get around it than a distant one.
    """
    gap = ir_to_cm(sensor_value)
    return ROBOT_CLEARANCE_CM + SAFETY_MARGIN_CM + max(0.0, SENSOR_RANGE_CM - gap)


# ---------------------------------------------------------------------------
# Helper: is there a wall on a given side? (uses the side + close-side sensors)
# ---------------------------------------------------------------------------
def side_blocked(sensors, side: str) -> bool:
    """True if the outer + close sensor on `side` ('left'/'right') see a wall."""
    if side == "left":
        return sensors[LEFT] >= WALL_TH or sensors[CLOSE_LEFT] >= WALL_TH
    return sensors[RIGHT] >= WALL_TH or sensors[CLOSE_RIGHT] >= WALL_TH


# ---------------------------------------------------------------------------
# Helper: grid cell of the obstacle in front of the robot
# ---------------------------------------------------------------------------
def obstacle_cell(pos, gap_cm: float) -> tuple:
    """Grid cell roughly `gap_cm` ahead of the robot along its heading."""
    heading = math.radians(getattr(pos, "heading", 0.0))
    ox = pos.x + math.cos(heading) * gap_cm
    oy = pos.y + math.sin(heading) * gap_cm
    return cm_to_grid(ox, oy)


def mark_barrier(gx: int, gy: int) -> bool:
    """Flip a free cell to a wall (1) in uGrid.  Returns True if it changed."""
    if 0 <= gy < len(uGrid) and 0 <= gx < len(uGrid[0]) and uGrid[gy][gx] == 0:
        uGrid[gy][gx] = 1
        return True
    return False


def reset_grid() -> None:
    """Restore uGrid to its pristine layout, dropping any temporary barriers."""
    global uGrid
    uGrid = copy.deepcopy(DEFAULT_GRID)


# ---------------------------------------------------------------------------
# Helper: slide sideways one direction, stopping if a wall blocks the step path
# ---------------------------------------------------------------------------
async def _try_side(robot, side: str, needed: float) -> float:
    """Turn toward `side`, slide up to `needed` cm, then square back up.

    While stepping, the FRONT sensor now points along the travel direction, so
    it doubles as a wall detector: if it trips WALL_TH the step stops short so
    the robot doesn't drive into a wall.  Returns the cm actually slid.
    """
    if side == "left":
        await robot.turn_left(90)
    else:
        await robot.turn_right(90)

    moved = 0.0
    limit = min(needed, MAX_SIDE_STEP_CM)
    while moved < limit:
        s = await read_ir_median(robot)
        if s[FRONT] >= WALL_TH:          # wall in the step path — stop short
            print(f"[AVOID] Wall while stepping {side} after {moved:.0f} cm.")
            break
        hop = min(SIDE_STEP_CM, limit - moved)
        await robot.move(hop)
        moved += hop

    # Return to the original heading regardless of how far we got.
    if side == "left":
        await robot.turn_right(90)
    else:
        await robot.turn_left(90)
    return moved


# ---------------------------------------------------------------------------
# Helper: doorway-style side-step around an obstacle (wall-aware, verified)
# ---------------------------------------------------------------------------
async def sidestep_obstacle(robot) -> str:
    """Go around an obstacle, verifying clearance and avoiding walls.

    Generalises the doorway demo's turn-90 / slide / turn-back maneuver:

      1. Re-read the sensors as a median (reject a noisy false alarm).
      2. Pick a side that isn't walled (side_blocked), preferring the more
         open diagonal.  How far to slide is set by clearance_distance().
      3. Slide via _try_side(), which stops short of any wall in the step path.
      4. Confirm with a fresh median read that the obstacle is actually gone
         before nudging forward past it (clearance_distance again).

    Returns
    -------
    str  "clear"   – re-check showed nothing there (false alarm, no maneuver).
         "avoided" – stepped around the obstacle and advanced past it.
         "blocked" – no room either side / still blocked (caller should mark
                     the grid and re-plan).
    """
    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_blink_rgb(255, 140, 0)            # amber = avoiding

    sensors = await read_ir_median(robot)

    # 1. Noisy false positive? Nothing really there — resume.
    if not obstacle_ahead(sensors):
        print("[AVOID] False alarm on re-check — path is clear.")
        await robot.set_lights_spin_rgb(0, 100, 255)
        return "clear"

    front_val = sensors[FRONT]
    needed    = clearance_distance(front_val)

    # 2. Choose which side(s) to try: only ones without a wall, more-open first.
    left_ok  = not side_blocked(sensors, "left")
    right_ok = not side_blocked(sensors, "right")
    prefer_left = sensors[DIAGONAL_LEFT] <= sensors[DIAGONAL_RIGHT]

    if left_ok and right_ok:
        order = ["left", "right"] if prefer_left else ["right", "left"]
    elif left_ok:
        order = ["left"]
    elif right_ok:
        order = ["right"]
    else:
        print("[AVOID] Walls on both sides — no room to side-step.")
        return "blocked"

    # 3. Try each candidate side; verify the obstacle is gone before advancing.
    for side in order:
        print(f"[AVOID] Side-stepping {side} (need ~{needed:.0f} cm).")
        await _try_side(robot, side, needed)

        check = await read_ir_median(robot)
        if not obstacle_ahead(check):
            # 4. Confirmed clear — move forward past the obstacle's depth.
            await robot.move(clearance_distance(front_val))
            await robot.set_lights_spin_rgb(0, 100, 255)
            print(f"[AVOID] Cleared obstacle to the {side}.")
            return "avoided"
        print(f"[AVOID] Still blocked after stepping {side}.")

    return "blocked"


# ---------------------------------------------------------------------------
# Helper: nearest waypoint on a path to the robot's current position
# ---------------------------------------------------------------------------
def closest_waypoint(pos, waypoints_cm: list, start: int = 0) -> int:
    """Index of the waypoint (at or after `start`) nearest to `pos`.

    Used after an obstacle detour to rejoin the original planned path: the
    side-step leaves the robot off-route, so we find the closest remaining
    waypoint and head there instead of fighting back to the one we were
    originally aiming at.  Restricting the search to `start` onward keeps the
    robot moving forward along the path rather than doubling back to a
    waypoint it has already passed.

    Parameters
    ----------
    pos          : robot position (has .x / .y in cm).
    waypoints_cm : full list of (x_cm, y_cm) waypoints — the original path.
    start        : lowest index to consider (waypoints before it are behind us).

    Returns
    -------
    int  Index into waypoints_cm of the nearest eligible waypoint.
    """
    best_i = start
    best_d = float("inf")
    for i in range(start, len(waypoints_cm)):
        wx, wy = waypoints_cm[i]
        d = math.hypot(wx - pos.x, wy - pos.y)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


# ---------------------------------------------------------------------------
# Helper: drive the whole path, avoiding obstacles and rejoining the route
# ---------------------------------------------------------------------------
async def navigate_path(robot, waypoints_cm: list) -> bool:
    """Drive along `waypoints_cm` while watching IR sensors for obstacles.

    The full waypoint list is kept intact for the whole leg so the robot can
    compare its position against it after a detour.  Each leg is driven by
    navigate_to() running as its own asyncio task (so it moves at full speed)
    while a concurrent loop polls the IR sensors every POLL_S seconds:

      - Path stays clear  → navigate_to() finishes → advance to next waypoint.
      - Obstacle detected → the navigate_to() task is cancelled and
                            sidestep_obstacle() tries to go around it; on
                            success the robot heads to the *closest remaining
                            waypoint on the original path* (closest_waypoint),
                            rejoining the planned route.
      - No room to pass   → the obstacle is marked as a barrier in uGrid
                            (mark_barrier) and a full re-plan is requested so
                            BFS routes around it.
      - Stuck             → after MAX_SIDESTEPS consecutive side-steps, mark a
                            barrier and re-plan.

    Parameters
    ----------
    robot        : Create3 robot instance.
    waypoints_cm : ordered list of (x_cm, y_cm) waypoints — the planned path.

    Returns
    -------
    bool  True if the final waypoint was reached.
          False if a re-plan is needed (bumper contact, or an obstacle that
          couldn't be cleared within MAX_SIDESTEPS).
    """
    global is_navigating, recalc_needed

    is_navigating = True
    index = 0
    sidesteps = 0
    try:
        while index < len(waypoints_cm):
            # A bumper contact (Layer-2 fallback) asks for an immediate re-plan.
            if recalc_needed:
                await robot.set_wheel_speeds(0, 0)
                return False

            target_x_cm, target_y_cm = waypoints_cm[index]

            # Already close enough to this waypoint? Move on to the next.
            pos = await robot.get_position()
            if math.hypot(target_x_cm - pos.x, target_y_cm - pos.y) <= ARRIVE_TOL_CM:
                index += 1
                sidesteps = 0
                continue

            # Drive the leg as a background task; poll sensors alongside it.
            nav_task = asyncio.ensure_future(
                robot.navigate_to(target_x_cm, target_y_cm))

            sensors = None
            while not nav_task.done():
                sensors = (await robot.get_ir_proximity()).sensors
                if obstacle_ahead(sensors) or recalc_needed:
                    break
                await asyncio.sleep(POLL_S)

            # navigate_to() reached the waypoint with a clear path → next one.
            if nav_task.done() and not nav_task.cancelled():
                await nav_task          # surface any navigation error
                index += 1
                sidesteps = 0
                continue

            # Obstacle (or bump) interrupted the drive — stop the motors.
            nav_task.cancel()
            try:
                await nav_task
            except asyncio.CancelledError:
                pass
            await robot.set_wheel_speeds(0, 0)

            if recalc_needed:
                return False

            # Try to go around the obstacle.
            status = await sidestep_obstacle(robot)
            pos = await robot.get_position()

            if status == "blocked":
                # No room to get around it — record the obstacle as a barrier
                # on the grid so BFS routes around it, then re-plan.
                blk      = await read_ir_median(robot)
                gap      = ir_to_cm(blk[FRONT])
                bx, by   = obstacle_cell(pos, gap + CELL_CM)
                if mark_barrier(bx, by):
                    print(f"[AVOID] No room — barrier marked at grid "
                          f"({bx},{by}); re-planning.")
                else:
                    print("[AVOID] No room — could not mark barrier; re-planning.")
                recalc_needed = True
                return False

            if status == "avoided":
                # A real detour — guard against endlessly dodging the same thing.
                sidesteps += 1
                if sidesteps > MAX_SIDESTEPS:
                    blk    = await read_ir_median(robot)
                    gap    = ir_to_cm(blk[FRONT])
                    bx, by = obstacle_cell(pos, gap + CELL_CM)
                    mark_barrier(bx, by)
                    print("[AVOID] Too many side-steps — marking barrier and "
                          "re-planning.")
                    recalc_needed = True
                    return False

            # "clear" (false alarm) or "avoided": rejoin the original path at
            # the nearest remaining waypoint (a detour may have carried us past
            # the one we were heading for).
            index = closest_waypoint(pos, waypoints_cm, index)

        return True
    finally:
        is_navigating = False


# ---------------------------------------------------------------------------
# Bumper handler — last-resort fallback if robot physically contacts something
# ---------------------------------------------------------------------------
@event(robot.when_bumped, [True, True])
async def on_bumped(robot):
    """React to any bumper contact during navigation.

    Backs up, turns away from the obstacle, then signals the main loop to
    recalculate the path from the robot's new position.
    """
    global recalc_needed, is_navigating

    if not is_navigating:
        return  # ignore bumps outside of active navigation

    # Stop immediately
    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_blink_rgb(255, 140, 0)   # amber blink = recalculating

    # Reverse away from the obstacle
    await robot.move(-BACKUP_CM)

    # Turn left 30° to clear the obstacle before re-planning
    await robot.turn_left(45)

    recalc_needed = True


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
    # queue = ["Veggies", "Meats", "Dairy"]
    # queue = ["Houseware", "Veggies", "Beverage", "Meats"]
    queue = ["Veggies", "Fruits", "Meats", "Pastries", "Condiments", "Canned", "Meals", "Snacks", "Cereal", "Houseware", "Dairy", "Beverage"]

    current_item  = None
    current_state = "AWAIT_ITEM"

    while True:
        await hand_over()   # yield to other events (bumper handler, etc.)

        # ==============================================================
        if current_state == "AWAIT_ITEM":

            # New item (or heading home): drop any temporary barriers marked
            # while routing the previous item so the map starts clean.
            reset_grid()

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

            # Keep the whole optimised path so navigate_path() can rejoin it
            # at the nearest waypoint after any obstacle detour.
            waypoints_cm = [grid_to_cm(wx, wy) for wx, wy in waypoints]
            print(f"[NAV] Following {len(waypoints_cm)}-waypoint path.")

            reached = await navigate_path(robot, waypoints_cm)

            if reached and not recalc_needed:
                current_state = "ARRIVED"
            else:
                print("[NAV] Obstacle blocked path — recalculating.")
                current_state = "PLAN_PATH"

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
                # await robot.play_note(Note.E5, 0.15)
                # await robot.play_note(Note.G5, 0.25)
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
                home_wpts    = optimize(home_raw)
                home_wpts_cm = [grid_to_cm(wx, wy) for wx, wy in home_wpts]
                await navigate_path(robot, home_wpts_cm)
            else:
                print("[ERROR] Cannot find path home.")

            pos = await robot.get_position()
            await robot.navigate_to(pos.x, pos.y, 90)
            await robot.set_lights_on_rgb(255, 255, 255)           # white = done
            # await robot.play_note(Note.C5, 0.2)
            # await robot.play_note(Note.G5, 0.2)
            # await robot.play_note(Note.C6, 0.4)
            print("[DONE] Back home. Queue complete.")
            break   # end the program


robot.play()
