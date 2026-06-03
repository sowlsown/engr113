"""
Supermarket directory robot — iRobot Create 3  (v3: wait-and-reroute, Home-anchored)
===================================================================================
Guides customers to items on a shopping list by navigating a 17×17
occupancy grid mapped to an 8 ft × 8 ft physical store.

This is a variant of supermarket.py.  Obstacle handling matches v2: the robot
does NOT estimate its way around anything — when it sees something ahead it
stops, waits to see if the obstacle moves, and if it doesn't, it blocks that
aisle on the grid and routes around the whole aisle.

v3 adds one thing over v2: the grid is anchored at Home, so the robot is
already "at Home" (1,1) on the grid the instant it starts and never has to
drive from (0,0) to (1,1) first.  See "Physical ↔ grid conversion" below.

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

  The grid is anchored at Home: coordinates are relative to Home's cell, so
  grid_to_cm(Home) == (0, 0) cm (the reset_navigation() origin).  The robot is
  therefore already "at Home" on the grid at start-up — no drive to (1,1).

  grid_to_cm(gx, gy) = ((gx - HOME_GX) * CELL_CM,  (gy - HOME_GY) * CELL_CM)

Obstacle handling (v2)
----------------------
  Detect → wait → block the aisle → reroute.  go_to() owns this loop for a
  single trip (one item, or the trip home):

  1. Detection (Layer 1):
       Each leg is driven by navigate_to() running as a background asyncio
       task while a concurrent loop polls the IR sensors every POLL_S seconds.
       If the front/diagonal sensors trip (obstacle_ahead), the drive is
       cancelled and the robot stops — it does NOT try to weave around it.

  2. Wait it out:
       wait_for_clear() holds position for up to WAIT_SECONDS, polling the
       (median-filtered) sensors.  If the object moves out of the way the
       robot simply resumes the same route.  People and carts move; shelves
       don't.

  3. Block the WHOLE aisle + reroute:
       If the object is still there after WAIT_SECONDS, the entire aisle it
       sits in is taken out of service — not just the one cell.  A ~2×2-cell
       robot can't pass an obstacle in a narrow aisle, so the whole shelf-
       flanked corridor (aisle_cells) is blocked and BFS routes around it via
       a different aisle.  Because the robot is *inside* that aisle when it
       detects the obstacle, it first reverses straight out to the cross-
       corridor (turning in a 2-wide aisle would risk a collision), then
       re-plans from there.  The destination cell and the cell the robot ends
       up on are never blocked, so it can't trap itself.

  Barrier lifetime — temporary vs permanent (per trip):
    * First time an obstacle is found in an aisle → TEMPORARY block.  It only
      influences the single reroute that follows; the next reroute rebuilds
      the grid without it, freeing that aisle again.  (So if the detour hits a
      different obstacle, the original aisle is available once more.)
    * If an obstacle is found again where one was already found this trip →
      PERMANENT block for the rest of this trip.
    * The grid is restored to its pristine layout (DEFAULT_GRID) only when the
      trip ends — i.e. once the robot reaches its destination and is ready for
      the next item.

  Layer 2 — bumper fallback (safety net):
    If something somehow makes contact, when_bumped fires, backs the robot up,
    and sets recalc_needed = True so the current trip re-plans from the new
    position.

  Sensor thresholds:
    WARN_TH  — front sensor: object ahead → stop and wait
    DIAG_TH  — diagonal sensors, slightly looser threshold

Light feedback
--------------
  Spinning blue   → navigating to next waypoint
  Solid green     → arrived at destination
  Blinking amber  → obstacle detected, waiting / rerouting
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

# Detection: navigate_to() drives the full leg while a concurrent task polls
# the IR sensors every POLL_S seconds, so the robot moves unhindered until
# something is actually detected ahead.
POLL_S        = 0.5   # seconds between obstacle checks during a drive
ARRIVE_TOL_CM = 5     # within this many cm of a waypoint counts as "reached"

# When something blocks the path, stop and watch it for up to WAIT_SECONDS;
# if it hasn't moved by then, treat it as a fixed obstacle and reroute.
WAIT_SECONDS = 4.0    # how long to wait for an obstacle to clear (3–5 s)
WAIT_POLL_S  = 0.5    # seconds between checks while waiting

# Sensors are noisy, so detection *decisions* use the median of several reads.
IR_SAMPLES      = 3     # IR snapshots taken per decision
IR_SAMPLE_GAP_S = 0.02  # delay between those snapshots

# Sensor thresholds — tune these to your physical environment.
# Higher raw IR value = closer object.
WARN_TH = 55     # front sensor: object ahead → stop and wait
DIAG_TH = 60     # diagonal sensor threshold

# An obstacle is assumed to block the whole aisle (the robot is ~2×2 cells, so
# it can't squeeze past one in a narrow corridor).  A free cell is treated as
# part of an "aisle" when the contiguous free run through it is this narrow or
# narrower on one axis; wider areas (the cross-corridors) are left open.
AISLE_WIDTH_MAX = 2

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

# Pristine copy of the store layout.  During a trip the robot blocks cells in
# uGrid where it finds fixed obstacles; the grid is restored from this default
# once the trip ends so those blocks don't leak into the next item.
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

# The robot's physical start position IS "Home".  Anchor the grid so that
# Home's cell maps to the reset_navigation() origin (0,0) cm.  That way the
# robot is already "at Home" on the grid the instant it starts — cm_to_grid(0,0)
# returns Home — so there's no need to drive from (0,0) to (1,1) first.
HOME_GX, HOME_GY, _, _ = DICTIONARY["Home"]

# ---------------------------------------------------------------------------
# Shared state between go_to(), the bumper handler, and the button handler
# ---------------------------------------------------------------------------
recalc_needed  = False      # bumper handler sets True; go_to() clears it
is_navigating  = False      # True while a navigate_to() call is in flight
advance_queue  = False      # right-button handler sets True when at ARRIVED


# ---------------------------------------------------------------------------
# Helper: grid ↔ cm conversion
# ---------------------------------------------------------------------------
def grid_to_cm(gx: int, gy: int) -> tuple:
    """Convert grid cell coordinates to physical cm for navigate_to().

    Coordinates are relative to Home: grid_to_cm(HOME) == (0, 0) cm, the
    reset_navigation() origin.
    """
    return ((gx - HOME_GX) * CELL_CM, (gy - HOME_GY) * CELL_CM)


def cm_to_grid(cx: float, cy: float) -> tuple:
    """Convert physical cm (relative to Home) back to the nearest grid cell."""
    return (round(cx / CELL_CM) + HOME_GX, round(cy / CELL_CM) + HOME_GY)


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

    A single IR snapshot is noisy and prone to spikes/dropouts; the median of
    a few rejects one-off outliers so we don't stop (or reroute) on a glitch.
    """
    rows = []
    for _ in range(samples):
        rows.append((await robot.get_ir_proximity()).sensors)
        await asyncio.sleep(IR_SAMPLE_GAP_S)
    return [statistics.median([r[i] for r in rows]) for i in range(len(rows[0]))]


# ---------------------------------------------------------------------------
# Helper: grid cell directly in front of the robot
# ---------------------------------------------------------------------------
def cell_ahead(pos) -> tuple:
    """The grid cell one cell-length ahead of the robot along its heading.

    This is the cell we block when a fixed obstacle is found: blocking a cell
    in a corridor forces BFS to route around that entire aisle.
    """
    heading = math.radians(getattr(pos, "heading", 0.0))
    ax = pos.x + math.cos(heading) * CELL_CM
    ay = pos.y + math.sin(heading) * CELL_CM
    return cm_to_grid(ax, ay)


# ---------------------------------------------------------------------------
# Helper: which way is the robot facing, as a cardinal grid step?
# ---------------------------------------------------------------------------
def heading_step(pos) -> tuple:
    """Robot heading rounded to a cardinal grid step: (1,0)/(-1,0)/(0,1)/(0,-1)."""
    heading = math.radians(getattr(pos, "heading", 0.0))
    cx, cy = math.cos(heading), math.sin(heading)
    if abs(cx) >= abs(cy):
        return (1 if cx >= 0 else -1, 0)
    return (0, 1 if cy >= 0 else -1)


# ---------------------------------------------------------------------------
# Helper: how "narrow" is the free corridor through a cell?
# ---------------------------------------------------------------------------
def _free_run(gx: int, gy: int, axis: str, grid: list) -> int:
    """Length of the contiguous free run through (gx, gy) along axis 'x' or 'y'."""
    rows, cols = len(grid), len(grid[0])
    if not (0 <= gy < rows and 0 <= gx < cols) or grid[gy][gx] != 0:
        return 0
    length = 1
    if axis == "x":
        x = gx - 1
        while x >= 0 and grid[gy][x] == 0:
            length += 1; x -= 1
        x = gx + 1
        while x < cols and grid[gy][x] == 0:
            length += 1; x += 1
    else:
        y = gy - 1
        while y >= 0 and grid[y][gx] == 0:
            length += 1; y -= 1
        y = gy + 1
        while y < rows and grid[y][gx] == 0:
            length += 1; y += 1
    return length


def _is_corridor(gx: int, gy: int, grid: list) -> bool:
    """True if the free cell sits in a narrow corridor (an aisle, not an open area)."""
    if not (0 <= gy < len(grid) and 0 <= gx < len(grid[0])) or grid[gy][gx] != 0:
        return False
    return min(_free_run(gx, gy, "x", grid),
               _free_run(gx, gy, "y", grid)) <= AISLE_WIDTH_MAX


# ---------------------------------------------------------------------------
# Helper: every cell of the aisle that contains a given cell
# ---------------------------------------------------------------------------
def aisle_cells(start_cell: tuple, grid: list) -> set:
    """Flood-fill the narrow corridor (aisle) containing `start_cell`.

    Expansion is limited to free cells that are themselves narrow corridors, so
    the fill spreads down the full length and width of the aisle but stops at
    the wide cross-corridors that join aisles together.  If `start_cell` isn't
    in a corridor (it's in an open area), a 2×2 footprint patch is returned
    instead so we still block the robot-sized spot.
    """
    sx, sy = start_cell
    rows, cols = len(grid), len(grid[0])
    if not (0 <= sy < rows and 0 <= sx < cols) or grid[sy][sx] != 0:
        return {start_cell}

    if not _is_corridor(sx, sy, grid):
        # Open area: just block a robot-sized (2×2) patch around the cell.
        return {(sx + ddx, sy + ddy) for ddx in (0, 1) for ddy in (0, 1)
                if 0 <= sy + ddy < rows and 0 <= sx + ddx < cols}

    seen = {(sx, sy)}
    stack = [(sx, sy)]
    while stack:
        x, y = stack.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if (nx, ny) not in seen and _is_corridor(nx, ny, grid):
                seen.add((nx, ny))
                stack.append((nx, ny))
    return seen


def cells_to_exit(rc: tuple, step: tuple, aisle: set) -> int:
    """How many cells the robot must reverse (opposite `step`) to leave `aisle`."""
    dx, dy = -step[0], -step[1]
    x, y = rc
    n = 0
    while (x, y) in aisle:
        x += dx; y += dy; n += 1
    return n


# ---------------------------------------------------------------------------
# Helper: rebuild uGrid from the pristine layout plus a set of blocked cells
# ---------------------------------------------------------------------------
def apply_barriers(cells) -> None:
    """Set uGrid to DEFAULT_GRID with every cell in `cells` marked blocked (1)."""
    global uGrid
    uGrid = copy.deepcopy(DEFAULT_GRID)
    for (gx, gy) in cells:
        if 0 <= gy < len(uGrid) and 0 <= gx < len(uGrid[0]):
            uGrid[gy][gx] = 1


def reset_grid() -> None:
    """Restore uGrid to its pristine layout, dropping all temporary blocks."""
    global uGrid
    uGrid = copy.deepcopy(DEFAULT_GRID)


# ---------------------------------------------------------------------------
# Helper: wait for an obstacle ahead to move out of the way
# ---------------------------------------------------------------------------
async def wait_for_clear(robot) -> bool:
    """Hold position for up to WAIT_SECONDS, watching the path ahead.

    Returns True as soon as a median read shows the path is clear (the object
    moved), or False if it is still blocked after WAIT_SECONDS (a fixed
    obstacle).
    """
    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_blink_rgb(255, 140, 0)        # amber = waiting

    waited = 0.0
    while waited < WAIT_SECONDS:
        sensors = await read_ir_median(robot)
        if not obstacle_ahead(sensors):
            await robot.set_lights_spin_rgb(0, 100, 255)  # back to blue
            return True
        await asyncio.sleep(WAIT_POLL_S)
        waited += WAIT_POLL_S

    return False


# ---------------------------------------------------------------------------
# Helper: drive a planned path, stopping (not steering) for obstacles
# ---------------------------------------------------------------------------
async def navigate_path(robot, waypoints_cm: list):
    """Drive along `waypoints_cm`, watching the IR sensors for obstacles.

    Each leg is driven by navigate_to() running as a background asyncio task
    while a concurrent loop polls the sensors every POLL_S seconds.  When an
    obstacle is detected the robot stops and waits (wait_for_clear); if the
    object moves it resumes the same route, otherwise it reports the blocking
    cell so the caller can block that aisle and reroute.

    Returns
    -------
    (str, cell)
      ("reached", None)   – final waypoint reached.
      ("blocked", (x, y)) – a fixed obstacle blocks the path at grid cell (x,y).
      ("replan", None)    – bumper contact; caller should re-plan.
    """
    global is_navigating, recalc_needed

    is_navigating = True
    index = 0
    try:
        while index < len(waypoints_cm):
            if recalc_needed:                       # bumper asked for a re-plan
                await robot.set_wheel_speeds(0, 0)
                return ("replan", None)

            target_x_cm, target_y_cm = waypoints_cm[index]

            # Already close enough to this waypoint? Move to the next one.
            pos = await robot.get_position()
            if math.hypot(target_x_cm - pos.x, target_y_cm - pos.y) <= ARRIVE_TOL_CM:
                index += 1
                continue

            # Drive the leg as a background task; poll sensors alongside it.
            nav_task = asyncio.ensure_future(
                robot.navigate_to(target_x_cm, target_y_cm))

            while not nav_task.done():
                sensors = (await robot.get_ir_proximity()).sensors
                if obstacle_ahead(sensors) or recalc_needed:
                    break
                await asyncio.sleep(POLL_S)

            # Reached this waypoint with a clear path → advance.
            if nav_task.done() and not nav_task.cancelled():
                await nav_task          # surface any navigation error
                index += 1
                continue

            # Something interrupted the drive — stop the motors.
            nav_task.cancel()
            try:
                await nav_task
            except asyncio.CancelledError:
                pass
            await robot.set_wheel_speeds(0, 0)

            if recalc_needed:
                return ("replan", None)

            # Obstacle ahead: stop and wait to see if it moves.
            print("[WAIT] Obstacle ahead — waiting up to "
                  f"{WAIT_SECONDS:.0f}s for it to move.")
            if await wait_for_clear(robot):
                print("[WAIT] Path cleared — resuming route.")
                continue            # re-issue navigate_to() to the same waypoint

            # Still blocked: report the cell ahead so the caller can reroute.
            pos  = await robot.get_position()
            cell = cell_ahead(pos)
            print(f"[BLOCK] Obstacle did not move — blocking aisle at {cell}.")
            return ("blocked", cell)

        return ("reached", None)
    finally:
        is_navigating = False


# ---------------------------------------------------------------------------
# Helper: plan + drive to a goal, rerouting around fixed obstacles
# ---------------------------------------------------------------------------
async def go_to(robot, goal_grid: tuple) -> bool:
    """Drive to goal_grid, rerouting around obstacles that refuse to move.

    Owns the barrier bookkeeping for one trip.  When an obstacle won't move the
    WHOLE aisle it sits in is taken out of service (a 2×2 robot can't pass an
    obstacle in a narrow aisle anyway), and the robot reverses straight out to
    the cross-corridor before re-planning (turning inside a 2-wide aisle would
    risk a collision):
      * temp_cells  – aisle blocked for the NEXT reroute only (one BFS).
      * seen_aisles – obstacle cells seen at least once this trip (memory for
                      detecting the same obstacle a second time).
      * perm_cells  – aisles blocked permanently for the rest of this trip
                      (an obstacle found again where one was already found).
    The pristine grid is restored when the trip ends (success or give-up), so
    the next item starts from a clean map.

    Returns True if the goal was reached, False if no route exists / gave up.
    """
    global recalc_needed

    perm_cells  = set()
    seen_aisles = set()
    temp_cells  = set()

    try:
        while True:
            recalc_needed = False

            # Build the map for this plan: pristine + permanent + one-shot temp.
            apply_barriers(perm_cells | temp_cells)
            temp_cells = set()      # consumed — a temp block lasts one reroute

            pos        = await robot.get_position()
            start_grid = cm_to_grid(pos.x, pos.y)
            print(f"[PATH] Planning {start_grid} → {goal_grid}")

            raw_path = find_path(start_grid, goal_grid, uGrid)
            if not raw_path:
                print(f"[ERROR] No route to {goal_grid}.")
                return False

            waypoints_cm = [grid_to_cm(wx, wy) for wx, wy in optimize(raw_path)]
            print(f"[NAV] Following {len(waypoints_cm)}-waypoint route.")
            await robot.set_lights_spin_rgb(0, 100, 255)   # blue = navigating

            status, cell = await navigate_path(robot, waypoints_cm)

            if status == "reached":
                return True

            if status == "replan":          # bumper contact — just re-plan
                continue

            # status == "blocked": take the obstacle's whole aisle out of service.
            if cell == goal_grid:
                print(f"[BLOCK] Obstacle on destination {cell}; cannot reach.")
                return False

            pos = await robot.get_position()
            rc  = cm_to_grid(pos.x, pos.y)

            # The whole shelf-flanked aisle the obstacle sits in.
            aisle = aisle_cells(cell, DEFAULT_GRID)

            # Reverse straight out of the aisle so it can be fully blocked
            # without trapping the robot, then re-read our position.
            back = cells_to_exit(rc, heading_step(pos), aisle)
            if back > 0:
                print(f"[BLOCK] Backing {back} cell(s) out of the aisle.")
                await robot.move(-back * CELL_CM)
                pos = await robot.get_position()
                rc  = cm_to_grid(pos.x, pos.y)

            # Don't block the destination or the cell we now stand on.
            barrier_set = set(aisle)
            barrier_set.discard(goal_grid)
            barrier_set.discard(rc)

            if cell in seen_aisles:
                perm_cells |= barrier_set   # second time here → permanent
                print(f"[BLOCK] Obstacle again near {cell} → aisle "
                      f"{sorted(barrier_set)} permanent for this trip.")
            else:
                seen_aisles.add(cell)
                temp_cells = barrier_set    # first time → temporary (one reroute)
                print(f"[BLOCK] Obstacle at {cell} → aisle "
                      f"{sorted(barrier_set)} out of service (temporary).")
            # loop back to re-plan around the blocked aisle
    finally:
        # Destination reached or trip abandoned: restore the clean map for the
        # next item in the queue.
        reset_grid()


# ---------------------------------------------------------------------------
# Bumper handler — last-resort fallback if robot physically contacts something
# ---------------------------------------------------------------------------
@event(robot.when_bumped, [True, True])
async def on_bumped(robot):
    """React to any bumper contact during navigation.

    Backs up, turns away, then signals the current trip to re-plan from the
    robot's new position.
    """
    global recalc_needed, is_navigating

    if not is_navigating:
        return  # ignore bumps outside of active navigation

    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_blink_rgb(255, 140, 0)   # amber blink = recalculating
    await robot.move(-BACKUP_CM)                    # reverse away from contact
    await robot.turn_left(45)                       # turn to clear it

    recalc_needed = True


# ---------------------------------------------------------------------------
# Right-button handler (••) — advance to next queue item
# ---------------------------------------------------------------------------
@event(robot.when_touched, [False, True])
async def on_button_advance(robot):
    """Skip the current wait and move to the next item in the queue.

    Only acts when the robot is stationary at a destination (ARRIVED).  A
    press mid-navigation is ignored so the robot can't be interrupted while
    moving.
    """
    global advance_queue, is_navigating

    if is_navigating:
        return  # robot is moving — ignore the press

    advance_queue = True
    await robot.set_lights_on_rgb(0, 100, 255)   # brief blue flash = acknowledged


# ---------------------------------------------------------------------------
# Main play loop
# ---------------------------------------------------------------------------
@event(robot.when_play)
async def play(robot):
    """Shopping-guide loop: for each queued item, drive there, wait for the
    customer to press (••), then continue.  When the queue is empty, go home.
    """
    global advance_queue

    # Reset odometry so (0,0) == robot start == "Home"
    await robot.reset_navigation()

    # Edit this list to change what the robot guides customers to.
    # Items must be keys in DICTIONARY.
    queue = ["Meats", "Meals", "Dairy"]

    while queue:
        await hand_over()   # yield to event handlers (bumper, button)

        item = queue.pop(0)
        print(f"[QUEUE] Next item: {item}")
        gx, gy, aisle_len, aisle_wid = DICTIONARY[item]

        # Drive there, rerouting around any aisle that's blocked by a fixed
        # obstacle.  go_to() restores the pristine grid when it returns.
        reached = await go_to(robot, (gx, gy))

        if not reached:
            print(f"[ERROR] Could not reach {item}. Skipping.")
            await robot.set_lights_blink_rgb(255, 0, 0)   # red = error
            await robot.wait(1.5)
            continue

        # --- Arrived: confirm we're in the stop zone, then announce. --------
        pos    = await robot.get_position()
        rx, ry = cm_to_grid(pos.x, pos.y)
        in_zone = (gx <= rx <= gx + aisle_len - 1 and
                   gy <= ry <= gy + aisle_wid - 1)

        if in_zone:
            print(f"[ARRIVED] At {item}!")
            await robot.play_note(Note.C5, 0.15)
        else:
            print(f"[ARRIVED] Near {item} "
                  f"(grid pos {rx},{ry}, zone origin {gx},{gy}). Continuing.")

        # --- Wait for the customer to press (••) to continue. ---------------
        advance_queue = False
        print("[WAIT] Press (••) to continue to next item.")
        while not advance_queue:
            await robot.set_lights_on_rgb(0, 220, 0)      # bright green
            await robot.wait(0.6)
            await robot.set_lights_on_rgb(0, 80, 0)       # dim pulse
            await robot.wait(0.4)
            await hand_over()                              # let button fire

        advance_queue = False
        print("[ADVANCE] Button pressed — moving to next item.")

    # --- Queue empty: head home. -------------------------------------------
    print("[HOME] All items done. Navigating home.")
    await robot.set_lights_spin_rgb(0, 100, 255)

    hx, hy, _, _ = DICTIONARY["Home"]
    if not await go_to(robot, (hx, hy)):
        print("[ERROR] Cannot find path home.")

    pos = await robot.get_position()
    await robot.navigate_to(pos.x, pos.y, 90)
    await robot.set_lights_on_rgb(255, 255, 255)          # white = done
    print("[DONE] Back home. Queue complete.")


robot.play()
