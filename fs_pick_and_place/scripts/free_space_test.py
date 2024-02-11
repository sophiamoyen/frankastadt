#!/usr/bin/env python3

import matplotlib.pyplot as plt

# Define axis limits
x_min = -9
x_max = 9
y_min = -9
y_max = 9

# Define grid spacing
grid_spacing = 4.5

# Generate x and y coordinates
x_coords = [x * grid_spacing for x in range(int(x_min // grid_spacing), int(x_max // grid_spacing))]
y_coords = [y * grid_spacing for y in range(int(y_min // grid_spacing), int(y_max // grid_spacing))]
print(y_coords)
print(x_coords)

poses = []
for x in x_coords:
    for y in y_coords:
        position = (x,y)
        poses.append(position)

# Create list of (x, y) poses

# Create the plot
plt.figure()

# Plot points
for x,y in poses:
    plt.plot(x,y, 'o', markersize=5, color='blue')

# Set axis limits and labels
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)
plt.xlabel("X (cm)")
plt.ylabel("Y (cm)")

# Customize plot appearance (optional)
plt.title("Points on Grid")
plt.grid(True)

plt.show()
