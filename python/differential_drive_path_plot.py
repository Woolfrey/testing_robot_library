#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 16 14:37:03 2025

@author: woolfrey
"""

import matplotlib.pyplot as plt
import os
import numpy as np

# Get the directory of the current script
script_dir = os.path.dirname(os.path.abspath(__file__))

# Build the full path to the CSV file
csv_path = os.path.join(script_dir, '..', 'build', 'desired_configuration_data.csv')

# Load the data
data = np.loadtxt(csv_path, delimiter=',')

# Columns: time, x, y, heading
time = data[:, 0]
x = data[:, 1]
y = data[:, 2]
heading = data[:, 3]  # Assuming the third value in each row is heading angle in radians

fig, ax = plt.subplots()

# Plot trajectory (x vs y)
ax.plot(x, y, label='Desired Path', color='black')

# Remove top and right spines
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# Remove grid
ax.grid(False)

# Add arrow at start position
start_arrow_length = 0.2
ax.arrow(
    x[0], y[0],
    start_arrow_length * np.cos(heading[0]),
    start_arrow_length * np.sin(heading[0]),
    head_width=0.05, head_length=0.1, fc='black', ec='black'
)

# Add arrow at end position
end_arrow_length = 0.2
ax.arrow(
    x[-1], y[-1],
    end_arrow_length * np.cos(heading[-1]),
    end_arrow_length * np.sin(heading[-1]),
    head_width=0.05, head_length=0.1, fc='black', ec='black'
)

# Set labels and title
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_title('Cartesian Path')

# Show legend
ax.legend()

plt.axis('equal')  # Equal aspect ratio so arrows look correct
plt.show()
