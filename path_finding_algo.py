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