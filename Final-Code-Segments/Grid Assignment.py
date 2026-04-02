import cv2
import numpy as np

def image_to_binary_grid(image_path, grid_rows, grid_cols, threshold=127):
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

    if img is None:
        raise ValueError("Image not found or could not be loaded.")

    height, width = img.shape

    cell_h = height // grid_rows
    cell_w = width // grid_cols

    grid = np.zeros((grid_rows, grid_cols), dtype=int)

    for i in range(grid_rows):
        for j in range(grid_cols):
            # Extract cell
            cell = img[i*cell_h:(i+1)*cell_h, j*cell_w:(j+1)*cell_w]

            # Compute average pixel intensity
            avg_intensity = np.mean(cell)

            # Assign value
            if avg_intensity < threshold:
                grid[i, j] = 1  # mostly black
            else:
                grid[i, j] = 0  # mostly white

    return grid
