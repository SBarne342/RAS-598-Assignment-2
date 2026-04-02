import cv2
import numpy as np

# Sets the print "cutoff" threshold to be indefinite so that the whole Grid array can be printed
np.set_printoptions(threshold=np.inf)

def image_to_binary_grid(image_path, grid_rows, grid_cols, proximity, threshold=127):
    """
    Converts an image into a grid of 0s and 1s.
    
    Parameters:
        image_path (str): Path to the image
        grid_rows (int): Number of rows in the grid
        grid_cols (int): Number of columns in the grid
        threshold (int): Pixel threshold to distinguish black/white (0–255)

    Returns:
        grid (2D numpy array): Grid of 0s (white) and 1s (black)
    """

    # Load image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # If no image is found then return a error message letting the user know
    if img is None:
        raise ValueError("Image not found or could not be loaded.")

    # Takes the dimensions of the image and seperates it into hieght and width variables
    height, width = img.shape

    # Assign cell hieght and width as the hieght/width of the image devided by the number of rows and columns respectively
    cell_h = height // grid_rows
    cell_w = width // grid_cols

    # Creates an array of all zeros that has the same dimentions as the number of rows and columns of the grid
    grid = np.zeros((grid_rows, grid_cols), dtype=int)

    # First for loop goes down the list of rows while the second goes through each column of each row
    for i in range(grid_rows):
        for j in range(grid_cols):
            # Takes the image and "removes" the cells that is row i column j
            # To do this it takes all pixels from i*cell_h to (j+1)*cell_w
            cell = img[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]

            # Compute average pixel intensity (0-255) using the mean funtion
            avg_intensity = np.mean(cell) 

            # Assign value based on the average
            if avg_intensity < threshold:
                grid[i, j] = 1  # mostly black
            else:
                grid[i, j] = 0  # mostly white

    # Debugging print used to see how the grid lines up with the image
    print(grid)


    # -----------------------------
    # Expand black cells to nearby cells
    # -----------------------------

    # Make a copy of the grid array to avoid modifying the orriginal while iterating
    expanded_grid = grid.copy()

    # To ensure that i and j are zero define them as such
    i = j = 0
    # First for loop goes down the list of rows while the second goes through each column of each row
    for i in range(grid_rows):
        for j in range(grid_cols):
            # checks if the cells is a 1 and if it is converts all 0's within the proximity to be a 1 too
            if grid[i, j] == 1:
                # Overwrites neighbors within 'proximity'
                # First for loop goes through every row within ∓proximity and the second for loop goes through every column within ∓proximity
                for di in range(-proximity, proximity+1):
                    for dj in range(-proximity, proximity+1):
                        # Assigns the row and column coordinates being potentially overwritten to ni and nj
                        ni, nj = i + di, j + dj
                        # Check boundaries of the grid so the grid array isn't accidentally expanded
                        if 0 <= ni < grid_rows and 0 <= nj < grid_cols:
                            # overwrites any 0's with a 1
                            expanded_grid[ni, nj] = 1

    grid = expanded_grid  # update the grid with expanded cells
    
    # Debugging print used to see how the grid lines up with the image
    print(grid)
    
    return grid

def main():
    # Calls the bit array function which converts the cave_filled.png image to an 80x80 array of 1's and 0's
    image_to_binary_grid("cave_filled.png", 80, 80, 1)

if __name__ == "__main__":
    main()
