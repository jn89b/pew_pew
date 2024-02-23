#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  3 17:10:00 2023

@author: justin
"""

#load pickle file
import pickle
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import os
import matplotlib.patches as patches

def get_state_control_info(solution):
    """get actual position and control info"""
    control_info = [np.asarray(control[0]) for control in solution]

    state_info = [np.asarray(state[1]) for state in solution]

    return control_info, state_info

def get_info_history(info: list, n_info: int) -> list:
    """get actual position history"""
    info_history = []

    for i in range(n_info):
        states = []
        for state in info:
            states.append(np.float64(state[i][0]))
        info_history.append(states)

    return info_history


def get_info_horizon(info: list, n_info: int) -> list:
    """get actual position history"""
    info_horizon = []

    for i in range(n_info):
        states = []
        for state in info:

            states.append(state[i, 1:])
        info_horizon.append(states)

    return info_horizon


#load pickle file from data 
curr_dir = os.getcwd()
print("Current directory: ", curr_dir)
folder_dir = curr_dir+'/data/'
pkl_name = 'level_flight_obstacles.pkl'
with open(folder_dir+pkl_name, 'rb') as handle:
    data = pickle.load(handle)

#load data
#remove first 10 seconds of data
index_parse = 5
control_ref_history = data['control_ref_history'][index_parse:]
obstacle_history = data['obstacle_history']

state_history = data['state_history'][index_parse:]
trajectory_ref_history = data['trajectory_ref_history'][index_parse:]
time_history = data['time_history'][index_parse:]
throttle_history = data['throttle_history'][index_parse:]
theta_pid = -np.rad2deg(data['theta_pid_history'][index_parse:])
goal = data['goal']

idx = 4
phi_rate_cmd = np.rad2deg([np.float64(control[0][idx]) for control in control_ref_history])
theta_rate_cmd = np.rad2deg([np.float64(control[1][idx]) for control in control_ref_history])
psi_rate_cmd = np.rad2deg([np.float64(control[2][idx]) for control in control_ref_history])
airspeed_cmd = [np.float64(control[3][idx]) for control in control_ref_history]

idx_trajectory = 4
x_trajectory = [np.float64(trajectory[0][idx_trajectory]) for trajectory in trajectory_ref_history]
y_trajectory = [np.float64(trajectory[1][idx_trajectory]) for trajectory in trajectory_ref_history]
z_trajectory = [np.float64(trajectory[2][idx_trajectory]) for trajectory in trajectory_ref_history]
phi_trajectory = np.rad2deg([np.float64(trajectory[3][idx_trajectory]) for trajectory in trajectory_ref_history])
theta_trajectory = np.rad2deg([np.float64(trajectory[4][idx_trajectory]) for trajectory in trajectory_ref_history])
psi_trajectory = np.rad2deg([np.float64(trajectory[5][idx_trajectory]) for trajectory in trajectory_ref_history])

#plot x y z
# go through list of lists and get the x y z position

state_ref_history = get_info_history(trajectory_ref_history, 6)

x_history = [state[0] for state in state_history]
y_history = [state[1] for state in state_history]
z_history = [state[2] for state in state_history]
phi_history = np.rad2deg([state[3] for state in state_history])
theta_history = np.rad2deg([state[4] for state in state_history])
psi_history = np.rad2deg([state[5] for state in state_history])
airspeed_history = [state[6] for state in state_history]


#plot x y z 3d plot
plt.close('all')
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_history, y_history, z_history, label='actual')

#plot start and end points
ax.scatter(x_history[0], y_history[0], z_history[0], c='r', marker='o', label='start')
ax.scatter(goal[0], goal[1], goal[2], c='g', marker='o', label='end')

#plot trajectory
ax.plot(x_trajectory, y_trajectory, z_trajectory, c='k', label='trajectory')

ax.set_xlabel('X meters')
ax.set_ylabel('Y meters')
ax.set_zlabel('Z meters')

#set legend
ax.legend()


#plot obstacles as cylinders
for obstacle in obstacle_history:
    x = obstacle[0]
    y = obstacle[1]
    z = 0
    radius = 50/2
    height = 100
    n_space = 10

    ax.plot_surface(
        x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
        color='g',
        alpha=0.2
    )

#set equal aspect ratio
ax.set_aspect('equal')

#%% Plot 3 subplot of roll pitch yaw

fig, axs = plt.subplots(6, 1)
#share x axis

fig.subplots_adjust(hspace=0)
axs[0].plot(time_history, phi_history, label='phi')
#axs[0].plot(time_history, phi_rate_cmd, label='phi rate cmd')
axs[0].plot(time_history, phi_trajectory, label='phi reference')
#set y label
axs[0].set_ylabel('phi (deg)')

axs[1].plot(time_history, theta_history, label='theta')
axs[1].plot(time_history, theta_pid, label='theta pid reference')
#axs[1].plot(time_history, theta_rate_cmd, label='theta rate cmd')
axs[1].plot(time_history, theta_trajectory, label='theta reference')
axs[1].set_ylabel('theta (deg)')


axs[2].plot(time_history, psi_history, label='psi')
axs[2].plot(time_history, psi_trajectory, label='psi reference')
#axs[2].plot(time_history, psi_rate_cmd, label='psi rate cmd')
axs[2].set_ylabel('psi (deg)')

axs[3].plot(time_history, airspeed_history, label='airspeed')
axs[3].plot(time_history, airspeed_cmd, label='airspeed cmd')
axs[3].set_ylabel('airspeed (m/s)')

axs[4].plot(time_history, throttle_history, label='throttle')
axs[4].set_ylabel('throttle')


dz_history = np.array(z_history) - goal[2]
axs[5].plot(time_history, dz_history, label='dz')
axs[5].set_ylabel('dz (m)')

#set legend
axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[3].legend()

#set grid 
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[3].grid()
axs[4].grid()
axs[5].grid()


#title of plot
fig.suptitle('Control and Reference Tracking')

#%% 
fig1, ax1 = plt.subplots(figsize=(8,8))
#set plot to equal aspect ratio
ax1.set_aspect('equal')
ax1.set_xlabel('x meters')
ax1.set_ylabel('y meters')

ax1.plot(x_history, y_history, 'o-')
ax1.plot(goal[0], goal[1], 'x')


for obstacle in obstacle_history:
    circle = patches.Circle((obstacle[0], obstacle[1]), obstacle[2]/2, 
        edgecolor='r', facecolor='none')
    ax1.add_patch(circle)


#%% Plot dz vs time
fig2, ax2 = plt.subplots()

ax2.set_xlabel('time (s)')
ax2.set_ylabel('dz (m)')
ax2.legend()


