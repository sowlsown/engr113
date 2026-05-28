import copy, shutil

class Pathfinding:
    _mag = 15.24
    _queue = [] # not quite sure what this does yet
    
    
    def __init__(self, grid = None):
        self.grid = sGrid if grid is None else grid
    
    def _find_path(self, start, goal, gx = 17, gy = 17):
        '''
        Find the shortest path between two points on the grid using BFS.
        This is an internal method.

        Parameters
        ----------
        start (tuple): tuple[int, int]
            (x, y) coordinates of the starting cell.
        goal (tuple): tuple[int, int]
            (x, y) coordinates of the destination cell.
        gx (int): optional
            Grid width (number of columns). Default is 17.
        gy (int): optional
            Grid height (number of rows). Default is 17.

        Returns
        -------
        list: list[tuple[int, int]]
            Ordered list of (x, y) cells from start to goal, inclusive.
            Returns an empty list if no path exists or the goal is blocked.
            
        Examples
        --------
        A simple path from (0, 0) to (2, 0) with no obstacles:
        >>> Pathfinding(sGrid)._find_path((0, 0), (2, 0))
        [(0, 0), (1, 0), (2, 0)]
        '''
        
        grid = copy.deepcopy(self.grid)
        Pathfinding._queue.append([start])
        visited = set([start])
        
        while Pathfinding._queue:
            path = Pathfinding._queue.pop(0)
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
                        Pathfinding._queue.append(new_path)
        return []
    
    def _optimize_path(self, path: list):
        '''
        Reduce the path by removing unnecessary points. A point is unnecessary if it is in a straight line with the previous and next points.
        This is an internal method.
        
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
        >>> Pathfinding(sGrid)._optimize_path([(0, 0), (1, 1)])
        [(0, 0), (1, 1)]

        A path where the middle point lies on a horizontal line is reduced:
        >>> Pathfinding(sGrid)._optimize_path([(0, 0), (1, 0), (2, 0)])
        [(0, 0), (2, 0)]

        A path where the middle point lies on a vertical line is reduced:
        >>> Pathfinding(sGrid)._optimize_path([(0, 0), (0, 1), (0, 2)])
        [(0, 0), (0, 2)]

        An L-shaped path keeps the corner point:
        >>> Pathfinding(sGrid)._optimize_path([(0, 0), (2, 0), (2, 3)])
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
    
    def route(self, start, goal):
        '''
        this does something idk ill write it later
        '''
        coordinates = self._optimize_path(self._find_path(start, goal))
        for i in range(len(coordinates)):
            x = coordinates[i][0] * self.magnitude
            y = coordinates[i][1] * self.magnitude
            coordinates[i] = (x, y)
        return coordinates
    
    
    def add_to_queue(self, destination):
        Pathfinding._queue.append(destination)
        return destination
    
    @property
    def magnitude(self):
        return self._mag
    
    @magnitude.setter
    def magnitude(self, value):
        if value <= 0:
            raise ValueError("Magnitude must be positive.")
        self._mag = value