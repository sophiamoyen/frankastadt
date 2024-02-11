#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math 


def find_free_space(occupied_positions, cube_size, lim_x, lim_y):
    """
    Finds free space for a group of cubes on the table with reserved space in each cube.

    Args:
        occupied_positions: List of tuples representing occupied positions (x, y) in cm.
        cube_size: Size of the cube in cm.
        lim_x: Tuble (x_min,x_max) of table
        lim_y: Tuple (y_min,y_max) of table

    Returns:
        List of tuples representing suggested free space positions (x, y) in cm.
    """
    free_space = []
    occupied_space = []
    x_min = lim_x[0]
    x_max = lim_x[1]
    y_min = lim_y[0]
    y_max = lim_y[1]

    x_coords = [x * cube_size for x in range(int(x_min // cube_size)+1, int(x_max // cube_size)+1)]
    y_coords = [y * cube_size for y in range(int(y_min // cube_size)+1, int(y_max // cube_size)+1)]

    for y in y_coords:
        for x in x_coords:
            grid_pos = (x,y)
            free = True
            
            for cube_pos in occupied_positions:
                safety_distance = (cube_size*math.sqrt(2))
                if math.dist(grid_pos,cube_pos) <= safety_distance:
                    free = False
                    break

            if free:
                free_space.append(grid_pos)

            if not free:
                occupied_space.append(grid_pos)


    return free_space, occupied_space

# Example usage (assuming the same data from the image)
occupied_positions = [[63, 26],[42,9],[29,-20],[63,-22],[43,-31]]
cube_size = 4.5  # cm
lim_y = [-80,80]
lim_x = [10,80]

free_positions, impossible_positions = find_free_space(occupied_positions, cube_size, lim_x, lim_y)

# Plot occupied and free spaces
fig, ax = plt.subplots(figsize=((10,8)))

for x, y in free_positions:
    # Plot free space
    plt.plot(y, x, 'o', markersize=6, color='lightgreen')
for x, y in impossible_positions:
    # Plot impossible positions
    plt.plot(y, x, 'o', markersize=6, color='red')
for x, y in occupied_positions:
    # Plot cubes
    square = patches.Rectangle((y-2.75, x-2.75), 4.5, 4.5, edgecolor='black', facecolor='blue')
    ax.add_patch(square)
    plt.gca().set_aspect('equal', adjustable='box')
    
plt.xlim(*lim_y)
plt.ylim(*lim_x)
plt.xlabel("Y (cm)")
plt.ylabel("X (cm)")
plt.title("Occupancy Grid")
plt.show()

