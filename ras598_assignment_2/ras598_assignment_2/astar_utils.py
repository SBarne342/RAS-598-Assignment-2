import heapq                                # priority queue implementation used for the A* open set (min-heap based on f-cost)
import math                                 # provides sqrt for distance calculations
from typing import Dict, List, Tuple        # type hints for readability and clarity
                                            # Dict: a built-in data structure used to store data in key-value pairs
                                            # List: used for type hinting to specify that a variable or return value is a list
                                            # Tuple: a built-in data structure used to store an ordered collection of items


def astar(
    grid,                           # 2D occupancy grid (typically a NumPy array); 0 = free space, nonzero = obstacle
    start: Tuple[int, int],         # (x, y) coordinates of the start cell
    goal: Tuple[int, int]           # (x, y) coordinates of the goal cell
) -> List[Tuple[int, int]]:         # returns a list of (x, y) cells representing the path
    
    # Extract grid dimensions
    #   Grid is indexed as grid[row, col] = grid[y, x]
    grid_height, grid_width = grid.shape

    """
    # is segment of code removed for being unnecessary
    def in_bounds(gx: int, gy: int) -> bool:
        # Checks whether a grid coordinate lies within map boundaries
        #   So, for example, if gx is bigger than the grid width, then it will return that it is out of bounds
        return 0 <= gx < grid_width and 0 <= gy < grid_height
    """


    # gx and gy are the grid x and y being used in the function
    def is_free(gx: int, gy: int) -> bool:
        # Returns True if the cell is within bounds AND not occupied
        # Convention: 0 = free, anything else = obstacle
        if ((0 <= gx < grid_width) and (0 <= gy < grid_height)) and (grid[gy, gx] == 0):
            return True
        else:
            return False

    # Unpack start and goal coordinates
    sx, sy = start              # Start position: start_x, start_y
    gx_goal, gy_goal = goal     # Goal grid position: grid_x_goal, grid_y_goal

    # Early exit if start or goal is invalid (e.g., inside obstacle)
    #   Just in case, so that way it hopefully won't cause further issues if it does happen
    if not is_free(sx, sy) or not is_free(gx_goal, gy_goal):
        print("Goal or Start position are within an obstacle. Check that grid assignment is oriented correctly")
        return []

    # Define motion model (8-connected grid)
    # Each tuple: (dx, dy, movement_cost)
    # Straight moves cost 1, diagonal moves cost sqrt(2)
    #   (x, y, cost)
    NEIGHBOURS = [
        ( 0,  1, 1.0),                # up
        ( 0, -1, 1.0),                # down
        ( 1,  0, 1.0),                # right
        (-1,  0, 1.0),                # left
        ( 1,  1, math.sqrt(2)),       # up-right
        ( 1, -1, math.sqrt(2)),       # down-right
        (-1,  1, math.sqrt(2)),       # up-left
        (-1, -1, math.sqrt(2)),       # down-left
    ]

    # A little unnecessary, but keeps the lines shorter
    def heuristic(ax: int, ay: int) -> float:
        # Euclidean distance heuristic (admissible for 8-connected grids)
        # Estimates cost from current node (ax, ay) to goal (gx_goal, gy_grid)
        return math.sqrt((ax - gx_goal) ** 2 + (ay - gy_goal) ** 2)

    # Open set (priority queue), storing (f_cost, node)
    # f = g + h (actual cost so far + heuristic estimate)
    open_heap = []      # the set of candidate robot states that the planner is still considering exploring
    heapq.heappush(open_heap, (heuristic(sx, sy), start))  # initialize with start node

    # Dictionary to reconstruct path: child --> parent
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    # g_cost (or goal_cost) stores the best known cost-to-come for each node
    # Initialized with start node at zero cost
    g_cost: Dict[Tuple[int, int], float] = {start: 0.0}

    # Initializes the closed set, which tracks nodes that have already been fully processed (expanded)
    closed: set = set()

    # Main A* search loop
    while open_heap:
        # Pop node with lowest f-cost
        #   Popped: It is removed from the priority queue (open_heap) and returned as the next node to process
        _, current = heapq.heappop(open_heap)

        # Skip if already processed (can happen due to duplicate insertions in heap)
        if current in closed:
            continue

        # Goal check
        if current == goal:
            # Reconstruct path by backtracking from goal to start
            path = [current]
            while current in came_from:
                current = came_from[current]    # move to parent
                path.append(current)            # add to the path list with the current position
            path.reverse()                      # reverse to get start -> goal ordering
            return path

        # Mark current node as expanded
        closed.add(current)
        cx, cy = current        # cx and cy are the current_x and current_y positions

        # Explore all neighbors
        #   dgx and dgy are the change in current grid positions or in other worlds the detal_grid_x and delta_grid_y
        for dgx, dgy, move_cost in NEIGHBOURS:
            # nx, ny = neighbour_x, neighbour_y
            nx, ny = cx + dgx, cy + dgy  # neighbor coordinates
            neighbour = (nx, ny)

            # Skip invalid or already explored nodes
            if not is_free(nx, ny) or neighbour in closed:
                continue

            # Prevent "corner cutting" during diagonal motion
            # Ensures robot does not pass through obstacle corners
            if abs(dgx) == 1 and abs(dgy) == 1:
                if not is_free(cx + dgx, cy) or not is_free(cx, cy + dgy):
                    continue

            # Compute tentative cost-to-come (g-value)
            # new heuristic g = g at current position + the cost of moving
            tentative_g = g_cost[current] + move_cost

            # If this path to neighbor is better than any previously found then
            if tentative_g < g_cost.get(neighbour, float('inf')):
                g_cost[neighbour] = tentative_g             # update best cost
                came_from[neighbour] = current              # update parent pointer

                f = tentative_g + heuristic(nx, ny)         # Compute f-cost for priority queue ordering

                heapq.heappush(open_heap, (f, neighbour))   # Push neighbor into open set (may create duplicates, but 
                                                            # is handled by closed set)

    # If open set is exhausted and goal was never reached, return empty path
    return []