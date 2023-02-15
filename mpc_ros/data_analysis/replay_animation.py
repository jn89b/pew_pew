#%% Visualize results 
#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
from matplotlib import animation
import seaborn as sns
import pandas as pd
import numpy as np
import ast

plt.close('all')


def convert_str_to_list(string_list):
    return ast.literal_eval(string_list)

# Import data from pandas
curr_dir = os.getcwd()
print("Current directory: ", curr_dir)
folder_dir = curr_dir+'/log/' 
df = pd.read_csv(folder_dir+'/test_13.csv')

# Remove any data that has empty lists
df = df[df != '[]']

# Remove any nan values
df = df.dropna()

#%% get horizon position array from dataframe
horizon_x_df = df['horizon_x']
horizon_y_df = df['horizon_y']
horizon_z_df = df['horizon_z']

overall_horizon_x = []
overall_horizon_y = []
overall_horizon_z = []

#convert list of strings to list of floats
for x,y,z in zip(horizon_x_df, horizon_y_df, horizon_z_df):
    #horizon_x = convert_str_to_list(x)
    overall_horizon_x.extend(convert_str_to_list(x))
    overall_horizon_y.extend(convert_str_to_list(y))
    overall_horizon_z.extend(convert_str_to_list(z))

horizon_pos_array = np.array([overall_horizon_x, 
                    overall_horizon_y, 
                    overall_horizon_z])

#%% Get the position of the aircraft
x_df = df['x']
y_df = df['y']
z_df = df['z']

x_position = []
y_position = []
z_position = []
for x,y,z in zip(x_df, y_df, z_df):
    x_position.append(x)
    y_position.append(y)
    z_position.append(z)

#actual
actual_pos_array = np.array([x_position, 
                    y_position, 
                    z_position])

#%% Plot the latency of response to system 
time = df['time']
plt.close('all')
#get every N value of overall_horizon_x
N = 15

x_list = []
y_list = []
z_list = []
index_desired = 0
for x,y,z in zip(horizon_x_df, horizon_y_df, horizon_z_df):
    x = convert_str_to_list(x)
    y = convert_str_to_list(y)
    z = convert_str_to_list(z)
    x_list.append(x[index_desired])
    y_list.append(y[index_desired])
    z_list.append(z[index_desired])
    
#create a 3 by 1 subplot
fig1, ax1 = plt.subplots(figsize=(6, 6), nrows=2, ncols=1, sharex=True)

end_idx = 150

#plot time and x position of aircraft in subplot 1
ax1[0].set_ylabel('x position (m)')
ax1[0].scatter(time[0:end_idx], x_position[0:end_idx], label='x')
ax1[0].scatter(time[0:end_idx], x_list[0:end_idx], label='end horizon x')
ax1[0].legend()

#plot time and y position of aircraft in subplot 2
ax1[1].set_ylabel('y position (m)')
ax1[1].scatter(time[0:end_idx], y_position[0:end_idx], label='y')
ax1[1].scatter(time[0:end_idx], y_list[0:end_idx], label='end horizon y')
#show legend
ax1[1].legend()


#%% Static Trajectory
min_x = min(x_position)
max_x = max(x_position)
buffer = 2


min_y = min(y_position)
max_y = max(y_position)

min_z = min(z_position)
max_z = max(z_position)

#need to map this as a parameter
x_target = 75.0
y_target = 75.0
z_target = 14

obstacle_x = 50
obstacle_y = 50
obstacle_z = 25
radius_obstacle = 10


fig3, ax3 = plt.subplots(figsize=(6, 6), subplot_kw=dict(projection='3d'))

ax3.set_xlim([min_x-buffer, max_x+buffer])
ax3.set_ylim([min_y-buffer, max_y+buffer])
ax3.set_zlim([0, 100])

u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
x = radius_obstacle*np.cos(u)*np.sin(v)
y = radius_obstacle*np.sin(u)*np.sin(v)
z = radius_obstacle*np.cos(v)
# alpha controls opacity
ax3.plot_surface(x+obstacle_x, y+obstacle_y, z+obstacle_z, color="g", alpha=0.3)

ax3.plot(x_position, y_position, z_position, label='Actual Position')
ax3.plot(overall_horizon_x, overall_horizon_y, overall_horizon_z, label='Horizon')
ax3.legend()

#%% Animate
x_init = x_position[0]
y_init = y_position[0]
z_init = z_position[0]



TIME_SPAN = 2
position_data = [actual_pos_array, horizon_pos_array]
labels = ['Actual Position', 'Horizon']


fig2, ax2 = plt.subplots(figsize=(6, 6), subplot_kw=dict(projection='3d'))

ax2.set_xlim([min_x-buffer, max_x+buffer])
ax2.set_ylim([min_y-buffer, max_y+buffer])
ax2.set_zlim([min_z-buffer, max_z+buffer])

ax2.scatter3D(obstacle_x, obstacle_y, 25)
ax2.plot(x_init, y_init, z_init, 'x', markersize=10, label='start')
ax2.plot(x_target, y_target, z_target, 'x', markersize=10,label='end')

lines = [ax2.plot([], [], [])[0] for _ in range(len(position_data))] 


color_list = sns.color_palette("hls", len(position_data))
# ax.scatter(5,5,1, s=100, c='g')
for i, line in enumerate(lines):
    line._color = color_list[i]
    #line._color = color_map[uav_list[i]]
    line._linewidth = 5.0
    line.set_label(labels[i])
    
patches = lines

def init():
    lines = [ax2.plot(uav[0, 0:1], uav[1, 0:1], uav[2, 0:1])[0] for uav in position_data]

    #scatters = [ax.scatter3D(uav[0, 0:1], uav[1, 0:1], uav[2, 0:1])[0] for uav in data]

    return patches

def update_lines(num, dataLines, lines):
    count = 0 
    for i, (line, data) in enumerate(zip(lines, dataLines)):
        #NOTE: there is no .set_data() for 3 dim data...
        time_span = TIME_SPAN
        if num < time_span:
            interval = 0
        else:
            interval = num - time_span
            
        if i == 1:
            line.set_data(data[:2, N*num:N*num+N])
            line.set_3d_properties(data[2, N*num:N*num+N])
        else:
            
            line.set_data(data[:2, interval:num])
            line.set_3d_properties(data[2, interval:num])
        
        count +=1
    
    return patches

# color_list = sns.color_palette("hls", num_columns)
# patches = set_lines(lines, color_list, column_names)

ax2.legend()
line_ani = animation.FuncAnimation(fig2, update_lines, fargs=(position_data, patches), init_func=init,
                                    interval=0.2, blit=True, repeat=True, frames=position_data[0].shape[1]+1)
