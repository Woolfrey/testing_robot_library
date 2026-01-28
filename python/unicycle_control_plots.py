#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import numpy as np

phi = 1.618                                                                                         # Golden ratio
this_directory = os.path.dirname(os.path.abspath(__file__))                                         # Local directory

####################################################################################################
#                                                PATH                                              #
####################################################################################################

# Load desired configuration data, skipping header
csv_path = os.path.join(this_directory, '..', 'build', 'unicycle_desired_configuration.csv')
desired_configuration = np.genfromtxt(csv_path, delimiter=',', names=True)

fig1, ax1 = plt.subplots(figsize=(8, 8))

# Plot the desired path
ax1.plot(desired_configuration['x_position_m'],
         desired_configuration['y_position_m'],
         label = "Desired Path",
         color = "black",
         linewidth = 2)
         
# Load desired configuration data, skipping header
csv_path = os.path.join(this_directory, '..', 'build', 'unicycle_actual_configuration.csv')
actual_configuration = np.genfromtxt(csv_path, delimiter=',', names=True)

# Plot the desired path
ax1.plot(actual_configuration['x_position_m'],
         actual_configuration['y_position_m'],
         label = "Actual Path",
         color = "red",
         linewidth = 2)
         
# Grab start and end positions
x_start, y_start = actual_configuration['x_position_m'][0], actual_configuration['y_position_m'][0]
x_end, y_end     = actual_configuration['x_position_m'][-1], actual_configuration['y_position_m'][-1]

# Grab start and end heading angles in radians
theta_start = actual_configuration['heading_angle_rad'][0]
theta_end = actual_configuration['heading_angle_rad'][-1]

arrow_length = 0.05  # meters, adjust as needed

# Compute arrow tip positions using heading
x_start_tip = x_start + arrow_length * np.cos(theta_start)
y_start_tip = y_start + arrow_length * np.sin(theta_start)

x_end_tip = x_end + arrow_length * np.cos(theta_end)
y_end_tip = y_end + arrow_length * np.sin(theta_end)

# Plot arrows
ax1.annotate('', xy=(x_start_tip, y_start_tip), xytext=(x_start, y_start),
             arrowprops=dict(facecolor='red', edgecolor='red', width=2, headwidth=8))

ax1.annotate('', xy=(x_end_tip, y_end_tip), xytext=(x_end, y_end),
             arrowprops=dict(facecolor='red', edgecolor='red', width=2, headwidth=8))

# Load obstacle data
csv_path = os.path.join(this_directory, '..', 'build', 'unicycle_obstacle_data.csv')
obstacle_data = np.genfromtxt(csv_path, delimiter=',', names=True)

# Grab the first obstacle (assuming it's static)
obs = obstacle_data[0]

centre = (obs['centre_x'], obs['centre_y'])
cov_matrix = np.array([[obs['xx'], obs['xy']],
                       [obs['xy'], obs['yy']]])

# Eigen decomposition
eigvals, eigvecs = np.linalg.eigh(cov_matrix)
order = eigvals.argsort()[::-1]  # Descending
eigvals = eigvals[order]
eigvecs = eigvecs[:, order]

# True width and height (no scaling)
width, height = 2 * np.sqrt(eigvals)

# Angle in degrees
angle = np.degrees(np.arctan2(eigvecs[1,0], eigvecs[0,0]))

# Create and add ellipse
ellipse = patches.Ellipse(xy = centre,
                          width = width,
                          height = height,
                          angle = angle,
                          edgecolor = [0.1, 0.1, 0.1],
                          facecolor = [0.2, 0.2, 0.2],
                          lw=2,
                          label='Obstacle')
ax1.add_patch(ellipse)


# --- Plot predicted horizon as dots ---
pred_x_path = os.path.join(this_directory, '..', 'build', 'unicycle_predicted_x.csv')
pred_y_path = os.path.join(this_directory, '..', 'build', 'unicycle_predicted_y.csv')

# Load last row, ignoring first column
predicted_x = np.loadtxt(pred_x_path, delimiter=',')[-1, 1:]
predicted_y = np.loadtxt(pred_y_path, delimiter=',')[-1, 1:]

# Plot predicted points as blue dots
ax1.scatter(predicted_x, predicted_y, color='red', marker='.', alpha=0.1, s=10, label='Predicted Horizon')

# Annotation and formatting
ax1.axis('equal')  # Ensure equal scaling for x and y
ax1.grid(False)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)', rotation=0, labelpad=20)
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
fig1.canvas.manager.set_window_title("Cartesian Path")


####################################################################################################
#                                         CONTROL INPUTS                                           #
####################################################################################################

# Load control inputs CSV
csv_path = os.path.join(this_directory, '..', 'build', 'unicycle_control_inputs.csv')
control_data = np.genfromtxt(csv_path, delimiter=',', names=True)

time = control_data['time_s']
linear_vel = control_data['linear_velocity_m_s']
angular_vel = control_data['angular_velocity_rad_s']

# Create a second figure for control inputs
fig2, (ax2_1, ax2_2) = plt.subplots(2, 1, figsize=(10, 6), sharex=False)

# ================== Top subplot: Linear velocity ==================
ax2_1.plot(time, linear_vel, color=[0.1, 0.1, 0.1], linewidth=2)
ax2_1.set_ylabel('Linear Velocity (m/s)')
ax2_1.grid(False)
ax2_1.spines['top'].set_visible(False)
ax2_1.spines['right'].set_visible(False)
ax2_1.spines['bottom'].set_visible(False)
ax2_1.tick_params(axis='x', which='both', bottom=False, labelbottom=False)  # hide ticks only for top subplot

# ================== Bottom subplot: Angular velocity ==================
ax2_2.plot(time, angular_vel * 30.0 / 3.1416, color=[0.1, 0.1, 0.1], linewidth=2)
ax2_2.set_ylabel('Angular Velocity (RPM)')
ax2_2.set_xlabel('Time (s)')
ax2_2.grid(False)
ax2_2.spines['top'].set_visible(False)
ax2_2.spines['right'].set_visible(False)

# Make sure ticks are visible on the bottom subplot
ax2_2.tick_params(axis='x', which='both', bottom=True, labelbottom=True)

# Optional: give the figure a window title
fig2.canvas.manager.set_window_title("Control Inputs")


####################################################################################################
#                                           TRACKING ERROR                                         #
####################################################################################################

# Time vector (assumed identical for desired and actual)
time = actual_configuration['time_s']

# Position error norm
pos_error = np.sqrt(
    (desired_configuration['x_position_m'] - actual_configuration['x_position_m'])**2 +
    (desired_configuration['y_position_m'] - actual_configuration['y_position_m'])**2
)

# Heading error (wrapped to [-pi, pi])
heading_error = np.arctan2(
    np.sin(desired_configuration['heading_angle_rad'] - actual_configuration['heading_angle_rad']),
    np.cos(desired_configuration['heading_angle_rad'] - actual_configuration['heading_angle_rad'])
)

# Create figure
fig3, (ax3_1, ax3_2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# ================== Position error ==================
ax3_1.plot(time, pos_error * 1000.0 , color=[0.1, 0.1, 0.1], linewidth=2)
ax3_1.set_ylabel('Position Error (mm)')
ax3_1.grid(False)
ax3_1.spines['top'].set_visible(False)
ax3_1.spines['right'].set_visible(False)
ax3_1.spines['bottom'].set_visible(False)
ax3_1.tick_params(axis='x', which='both', bottom=False, labelbottom=False)

# ================== Heading error ==================
ax3_2.plot(time, heading_error * 180.0 / 31.1416, color=[0.1, 0.1, 0.1], linewidth=2)
ax3_2.set_ylabel('Heading Error (deg)')
ax3_2.set_xlabel('Time (s)')
ax3_2.grid(False)
ax3_2.spines['top'].set_visible(False)
ax3_2.spines['right'].set_visible(False)
ax3_2.tick_params(axis='x', which='both', bottom=True, labelbottom=True)

# Window title
fig3.canvas.manager.set_window_title("Tracking Error")


plt.tight_layout()
plt.show()
