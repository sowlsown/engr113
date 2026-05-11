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
    """Find a path from a start cell to a goal cell using breadth-first search.

    Parameters
    ----------
    start : tuple[int, int]
        Starting coordinate as ``(x, y)``.
    goal : tuple[int, int]
        Goal coordinate as ``(x, y)``.
    grid : list[list[int]]
        Grid map where ``0`` represents an open cell and ``1`` represents a wall.

    Returns
    -------
    list[tuple[int, int]]
        The path from ``start`` to ``goal`` as a list of coordinates, or an empty
        list if no path exists.

    Examples
    --------
    >>> grid = [
    ...     [0, 0, 0],
    ...     [0, 0, 0],
    ...     [0, 0, 0],
    ... ]
    >>> start = (0, 0)
    >>> goal = (2, 0)
    >>> path = find_path(start, goal, grid)
    >>> for y in range(3):
    ...     row = ""
    ...     for x in range(3):
    ...         if (x, y) == start:
    ...             row += " S "
    ...         elif (x, y) == goal:
    ...             row += " G "
    ...         elif (x, y) in path:
    ...             row += " * "
    ...         else:
    ...             row += " . "
    ...     print(row)
     S  *  G 
     .  .  . 
     .  .  . 
    """
    
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
if __name__ == "__main__":
    start_pos = (0, 7)
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

