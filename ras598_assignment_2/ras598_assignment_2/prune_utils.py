from typing import List, Tuple  # Provides type hints for readability and static analysis

    # Determines whether there is a collision-free straight-line path between 'start' and 'end' on a 2D occupancy grid.
def _line_of_sight(
    grid,                       # grid   : 2D numpy array (0 = free, 1 = occupied)
    start: Tuple[int, int],     # start  : (x, y) integer grid coordinate of starting cell
    end: Tuple[int, int],       # end    : (x, y) integer grid coordinate of ending cell
    margin: int = 0             # margin : inflation radius (in cells) around obstacles for safety
) -> bool:

    grid_height, grid_width = grid.shape    # Extract grid dimensions
    
    # Returns True if a cell is considered occupied
    #   Out-of-bounds is treated as occupied to enforce safety at edges
    def occupied(gx: int, gy: int) -> bool:
        if not (0 <= gx < grid_width and 0 <= gy < grid_height):    # Checks whether a grid coordinate lies within map bounds
            return True
        return grid[gy, gx] == 1  # Note indexing order: (row=y, col=x)
    
    # Checks whether a cell and its surrounding neighborhood (defined by margin) are all free. 
    # This effectively inflates obstacles by 'margin'.
    def check_cell(gx: int, gy: int) -> bool:
        for dx in range(-margin, margin + 1):
            for dy in range(-margin, margin + 1):
                if occupied(gx + dx, gy + dy):
                    return False                    # Any occupied cell in the neighborhood blocks the path
        return True

    x0, y0 = start  # Start coordinates
    x1, y1 = end    # End coordinates

    # Compute absolute differences in x and y directions
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    # Step direction (+1 or -1) for x and y
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy    # Bresenham-style error term initialization
    cx, cy = x0, y0    # Current point initialized to start

    while True:
        # Check whether the current cell (with margin) is collision-free
        if not check_cell(cx, cy):
            return False  # Early exit if any cell along the line is invalid

        # Stop when we reach the end cell
        if cx == x1 and cy == y1:
            break

        e2 = 2 * err        # Bresenham line stepping logic

        # Move in x-direction if error condition satisfied
        if e2 > -dy:
            err -= dy
            cx  += sx

        # Move in y-direction if error condition satisfied
        if e2 < dx:
            err += dx
            cy  += sy

    return True      # returns: True if the straight line is collision-free, False otherwise


# Reduces the number of waypoints in a path by removing unnecessary intermediate nodes.
def prune_path(
    grid,                           # grid   : occupancy grid (same format as above)
    path: List[Tuple[int, int]],    # path   : list of (x, y) waypoints (typically from A*, Dijkstra, etc.)
    margin: int = 0                 # margin : obstacle inflation for safety during pruning
) -> List[Tuple[int, int]]:
    
    if not path: return []        # Handle empty input path

    if len(path) <= 2: return list(path)        # Nothing to prune if path has 0, 1, or 2 points

    pruned = [path[0]]  # Always keep the starting point
    current_idx = 0     # Index of the current waypoint in the original path

    while current_idx < len(path) - 1:    # Iterate until we reach the final waypoint

        farthest_idx = len(path) - 1        # Start by assuming we can jump directly to the goal

        while farthest_idx > current_idx + 1:        # Walk backward to find the farthest reachable waypoint
            # Check if a direct connection is collision-free
            if _line_of_sight(grid, path[current_idx], path[farthest_idx], margin):
                break               # Found the farthest valid shortcut
            
            farthest_idx -= 1       # Otherwise, try a closer waypoint

        pruned.append(path[farthest_idx])        # Append the selected waypoint to the pruned path

        current_idx = farthest_idx        # Move current index forward to the chosen waypoint
        
    return pruned      # returns: a new pruned path with fewer waypoints