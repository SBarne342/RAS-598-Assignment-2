import cv2
import json
import numpy as np

# Sets the print "cutoff" threshold to be indefinite so that the whole Grid array can be printed
np.set_printoptions(threshold=np.inf)

class PipelineConfig:
    """
    Holds parameters for the point cloud processing pipeline.
    """
    def __init__(self):
        # Image to binary grid configurables
        self.image_path = "cave_filled.png"     # image location
        self.grid_rows = 80                     # number of rows in the grid (aquired from 16.0m / 0.2m)
        self.grid_cols = 80                     # number of columns in the grid (aquired from 16.0m / 0.2m)
        self.proximity = 1                      # the number of cells that will be "bloated" to account for the size of the robot
        self.threshold = 127                    # the threshold for a cell to be marked as occupied

        # Print configurables
        self.json_out = "results.json"
        

class Pipeline:

    def __init__(self, cfg: PipelineConfig):
        self.cfg = cfg      # Set "self.cfg" to point back to PipelineConfig class

    def image_to_binary_grid(self, image_path, grid_rows, grid_cols, proximity, threshold=127):
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

        # Debugging print use to see how the grid lines up with the image
        print(grid)


        # -----------------------------
        # Expand black cells to nearby cells
        # -----------------------------

        # Make a copy of the grid array to avoid modifying the orriginal while iterating
        expanded_grid = grid.copy()

        i = j = 0 # To ensure that i and j are zero define them as such
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
        
        # Debugging print use to see how the grid lines up with the image
        print(grid)

        return grid

    def JSON_Write(self, grid):
        h = 400
        w = 400
        grid_h = h / self.cfg.grid_rows
        grid_w = w / self.cfg.grid_cols
        cells = []
        for gy in range(self.cfg.grid_rows):
            y0 = gy * grid_h
            y1 = min((gy + 1) * grid_h, h)
            cell_y = int((y0 + y1) / 2)
            for gx in range(self.cfg.grid_cols):
                x0 = gx * grid_w
                x1 = min((gx + 1) * grid_w, w)
                cell_x = int((x0 + x1) / 2)
                value = grid[gy, gx]
                cells.append({
                    "row": int(gy),
                    "col": int(gx),
                    "value": int(value),
                    "center_px": [int(cell_x), int(cell_y)]
                })


            """
            Task Service: Call the /get_task service to receive start and goal coordinates. This resets your energy counter. The response is
            a string of the format "{start_x},{start_y},{goal_x},{goal_y}"
            """
            meta = {
            # "input": args.input,
            # "circles_overlay_path": args.circles_overlay_out,
            # "grid_size_px": grid,
            # "grid_rows": int(gh),
            # "grid_cols": int(gw),
            # "threshold_percent": args.threshold,
            # "grid_overlay_path": args.grid_overlay_out,
            # "grid_overlay_annot_path": args.grid_overlay_annot_out,
            # "walls_mask_path": args.walls_mask_out,
            # "circles": circles_info,
            "cells": cells
        }
        
        # Open file open(file_description_or_path, open_text_mode)
        #    open_text_mode: "w" = Opens for writing, overwriting existing content
        with open(self.cfg.json_out, "w") as f:
            json.dump(meta, f, indent=2) # f: A file-like object opened for writing
                                         # indent=2: Specifies the number of spaces to use for each level of indentation

    def main(self):
        # Calls the bit array function which converts the cave_filled.png image to an 80x80 array of 1's and 0's
        grid = self.image_to_binary_grid(
            self.cfg.image_path, 
            self.cfg.grid_rows, 
            self.cfg.grid_cols,
            self.cfg.proximity, 
            self.cfg.threshold
            )
        
        self.JSON_Write(grid)

if __name__ == "__main__":
    cfg = PipelineConfig()
    pipeline = Pipeline(cfg)
    pipeline.main()
