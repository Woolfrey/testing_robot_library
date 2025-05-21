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

# Load desired configuration data
desired_csv_path = os.path.join(script_dir, '..', 'build', 'desired_configuration_data.csv')
desired_data = np.loadtxt(desired_csv_path, delimiter=',')
time = desired_data[:, 0]
x_desired = desired_data[:, 1]
y_desired = desired_data[:, 2]
heading_desired = desired_data[:, 3]

# Load actual configuration data
actual_csv_path = os.path.join(script_dir, '..', 'build', 'actual_configuration_data.csv')
actual_data = np.loadtxt(actual_csv_path, delimiter=',')
x_actual = actual_data[:, 1]
y_actual = actual_data[:, 2]
heading_actual = actual_data[:, 3]

# Plot desired and actual paths
fig1, ax1 = plt.subplots()

ax1.plot(x_desired, y_desired, label='Desired', color='black')
ax1.plot(x_actual,  y_actual,  label='Actual',  color='red')

# Arrows for desired path start and end
arrow_length = 0.05
ax1.arrow(x_actual[0], y_actual[0],
          arrow_length * np.cos(heading_actual[0]),
          arrow_length * np.sin(heading_actual[0]),
          head_width=0.03, head_length=0.05, fc='red', ec='red')

ax1.arrow(x_actual[-1], y_actual[-1],
          arrow_length * np.cos(heading_actual[-1]),
          arrow_length * np.sin(heading_actual[-1]),
          head_width=0.03, head_length=0.05, fc='red', ec='red')

# Style adjustments
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.grid(False)
ax1.set_xlabel('X Position')
ax1.set_ylabel('Y Position')
ax1.set_title('Cartesian Path')
ax1.legend()
ax1.axis('equal')

# Load control input data
control_csv_path = os.path.join(script_dir, '..', 'build', 'control_input_data.csv')
control_data = np.loadtxt(control_csv_path, delimiter=',')
control_time = control_data[:, 0]
linear_velocity = control_data[:, 1]
angular_velocity = control_data[:, 2]

# Plot control inputs in subfigures
fig2, (ax2_1, ax2_2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# Linear velocity
ax2_1.plot(control_time, linear_velocity, color='black')
ax2_1.set_ylabel('Linear Velocity (m/s)')
ax2_1.spines['top'].set_visible(False)
ax2_1.spines['right'].set_visible(False)
ax2_1.spines['bottom'].set_visible(False)
ax2_1.tick_params(axis='x', which='both', bottom=False, top=False)
ax2_1.grid(False)

# Angular velocity
ax2_2.plot(control_time, angular_velocity, color='black')
ax2_2.set_ylabel('Angular Velocity (rad/s)')
ax2_2.set_xlabel('Time (s)')
ax2_2.spines['top'].set_visible(False)
ax2_2.spines['right'].set_visible(False)
ax2_2.grid(False)
fig2.suptitle('Control Inputs')

# Load tracking error data
error_csv_path = os.path.join(script_dir, '..', 'build', 'tracking_error_data.csv')
error_data = np.loadtxt(error_csv_path, delimiter=',')
error_time = error_data[:, 0]
position_error_mm = error_data[:, 1] * 1000  # Convert meters to mm
orientation_error_deg = np.degrees(error_data[:, 2])  # Convert radians to degrees

# Plot tracking errors in subfigures
fig3, (ax3_1, ax3_2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

# Position error
ax3_1.plot(error_time, position_error_mm, color='black')
ax3_1.set_ylabel('Position Error (mm)')
ax3_1.spines['top'].set_visible(False)
ax3_1.spines['right'].set_visible(False)
ax3_1.spines['bottom'].set_visible(False)
ax3_1.tick_params(axis='x', which='both', bottom=False, top=False)
ax3_1.grid(False)

# Orientation error
ax3_2.plot(error_time, orientation_error_deg, color='black')
ax3_2.set_ylabel('Orientation Error (°)')
ax3_2.set_xlabel('Time (s)')
ax3_2.spines['top'].set_visible(False)
ax3_2.spines['right'].set_visible(False)
ax3_2.grid(False)
fig3.suptitle('Tracking Errors')

plt.tight_layout()
plt.show()

plt.tight_layout()
plt.show()
