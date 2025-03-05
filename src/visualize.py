import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# File path
occupancy_grid_file_path = "inputAndOutputTable/output.csv"

# Load the occupancy grid from the CSV file
occupancy_grid = np.genfromtxt(occupancy_grid_file_path, delimiter=',', filling_values=2.0)

# Create a larger figure but maintain proportions
plt.figure(figsize=(7, 7), dpi=200)

# Plot the occupancy grid
plt.imshow(occupancy_grid, cmap='Greys', interpolation='none', origin='lower', aspect='auto')
plt.colorbar(label='Occupancy')
plt.title('Occupancy Grid')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')

# Show the plot
plt.show()
