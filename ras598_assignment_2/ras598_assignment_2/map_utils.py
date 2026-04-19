import math                 # provides mathematical functions like sqrt and ceil used for geometry and discretization
import numpy as np          # numerical library for efficient array operations (used heavily for grid representation)
from PIL import Image       # used to load and manipulate image files (map input)


# Loads a grayscale image from disk and converts it into a NumPy array
def load_map_image(map_path: str) -> np.ndarray:    # map_path: string path to the map image file (e.g., .png)
    image = Image.open(map_path).convert('L')       # open image and convert to grayscale ('L' mode = 8-bit pixels)
    return np.array(image)                          # returns: 2D numpy array where each element is an intensity value [0, 255]


# Converts a grayscale map image into a binary occupancy grid at a desired resolution
def build_occupancy_grid(
    image: np.ndarray,              # image: 2D numpy array (grayscale map)
    source_resolution: float,       # source_resolution: meters per pixel of the original image (map scale)
    target_resolution: float        # target_resolution: meters per cell for the output occupancy grid
) -> np.ndarray:
    
    image_height, image_width = image.shape  # extract dimensions of input image (rows, cols)

    # Convert grayscale image into binary occupancy:
    # pixels darker than 128 are considered obstacles (1), others are free (0)
    binary = np.where(image < 128, 1, 0).astype(np.uint8)

    # Compute physical dimensions of the map in meters
    map_width_m  = image_width  * source_resolution   # map width in meters = image width x meters per pixel
    map_height_m = image_height * source_resolution   # map height in meters = image height x meters per pixel

    # Compute size of output grid based on desired resolution
    # ceil ensures full coverage of the map (no truncation)
    grid_width  = int(math.ceil(map_width_m  / target_resolution))
    grid_height = int(math.ceil(map_height_m / target_resolution))

    # Initialize empty occupancy grid (all free space initially)
    grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Iterate through every pixel in the input image
    # Going first through every column in a row before moving to the next
    for img_row in range(image_height):
        for img_col in range(image_width):
            # Only process pixels classified as obstacles
            if binary[img_row, img_col] == 1:

                # Convert image column index to grid column (gc) index
                # scales from source resolution to target resolution
                gc = int(img_col * source_resolution / target_resolution)

                # Convert image row index to world/grid row index
                # image origin is top-left, but occupancy grid uses bottom-left origin
                world_row_from_bottom = image_height - 1 - img_row

                # scale to grid resolution
                gr = int(world_row_from_bottom * source_resolution / target_resolution)

                # Ensure computed indices fall within grid bounds
                # gr = grid_row and gc = grid_column
                if 0 <= gr < grid_height and 0 <= gc < grid_width:
                    grid[gr, gc] = 1  # mark this grid cell as occupied

    return grid     # returns: 2D numpy array (occupancy grid), where 1 = obstacle, 0 = free space

# Expands obstacles in the occupancy grid to account for robot size and safety margin
def inflate_obstacles(grid: np.ndarray, inflation_cells: int) -> np.ndarray:
    # grid: 2D numpy array (occupancy grid), where 1 = obstacle, 0 = free
    # inflation_cells: radius (in grid cells) to inflate obstacles

    grid_height, grid_width = grid.shape  # dimensions of the occupancy grid

    inflated = grid.copy()  # create a copy so original grid is not modified

    # Find coordinates of all occupied cells (obstacles)
    # returns array of (row, col) indices where grid == 1
    occupied = np.argwhere(grid == 1)

    # For each occupied cell, expand its influence to nearby cells
    for (r, c) in occupied:
        # iterate over a square neighborhood centered at (r, c)
        for dr in range(-inflation_cells, inflation_cells + 1):
            for dc in range(-inflation_cells, inflation_cells + 1):

                # compute neighbor cell coordinates
                # dr = how far you move vertically (row direction)
                # dc = how far you move horizontally (column direction)
                # neighbour_row, neighbour_column = row + directory_row, column + directory_column
                nr, nc = r + dr, c + dc     

                # check that neighbor is within grid bounds
                if 0 <= nr < grid_height and 0 <= nc < grid_width:
                    # only inflate within a circular radius (Euclidean distance)
                    #   this avoids square-shaped inflation and better approximates robot footprint
                    if math.sqrt(dr * dr + dc * dc) <= inflation_cells:
                        inflated[nr, nc] = 1  # mark neighbor cell as occupied

    return inflated  ## returns: new grid where obstacles are "grown" outward
