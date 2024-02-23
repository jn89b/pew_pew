import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import os 
import ast
import numpy as np
from matplotlib.animation import FuncAnimation

conversion = lambda df: ast.literal_eval(df)

def parse_trajectory(trajectory_df):
    #remove 'array' from the string
    trajectory_df = trajectory_df.str.replace('array', '')
    trajectory_df.dropna(inplace=True)
    trajectory = [ ]
    for traj in trajectory_df:
        if traj == 'nan':
            continue
        val = ast.literal_eval(traj)
        trajectory.append(val[1])

    return trajectory


plt.close('all')
#set aesthetics
sns.set_style("whitegrid")
sns.set_context("paper")

cwd = os.getcwd()
print(cwd)
df = pd.read_csv('dumpster_log.csv')

obstacles_df = df['obstacles']
obstacles_df = obstacles_df.dropna()
obstacles = []
for obstacle in obstacles_df:
    obstacles.append(ast.literal_eval(obstacle))

df = df.drop(columns='obstacles')
# df = df.dropna()
#df = pd.read_csv(cwd+cod'/log/obstacle_avoidance_two.csv')
#swap x and y
enu_x_cmd = df['y_cmd']
enu_y_cmd = df['x_cmd']
enu_z_cmd = df['z_cmd']
enu_roll_cmd = df['roll_cmd']
enu_pitch_cmd = df['pitch_cmd']
enu_yaw_cmd = df['yaw_cmd']

y_traj = parse_trajectory(df['x_traj'])
x_traj = parse_trajectory(df['y_traj'])
z_traj = parse_trajectory(df['z_traj'])
roll_traj = parse_trajectory(df['roll_traj'])
pitch_traj = parse_trajectory(df['pitch_traj'])
yaw_traj = parse_trajectory(df['yaw_traj'])


##### PLOTS #####
#plot the obstacles and the trajectory
fig,ax = plt.subplots()
ax.scatter(df['x'], df['y'], label='position')
ax.scatter(enu_x_cmd, enu_y_cmd, color='g', label='command')

for obstacle in obstacles:
    circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2]/2, color='r')
    ax.add_artist(circle)

ax.legend()
plt.show()

fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')
for obstacle in obstacles:
    x = obstacle[0]
    y = obstacle[1]
    z = 0
    radius = obstacle[2]/2
    height = 50
    n_space = 10

    ax2.plot_surface(
        x + radius * np.outer(np.cos(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        y + radius * np.outer(np.sin(np.linspace(0, 2 * np.pi, n_space)), np.ones(n_space)),
        z + height * np.outer(np.ones(n_space), np.linspace(0, 1, n_space)),
        color='g',
        alpha=0.5
    )
ax2.plot(df['x'], df['y'], df['z'], label='position', color='b')

#%%
### figure 3 ### 
fig3, ax3 = plt.subplots(subplot_kw=dict(projection="3d"))
ax3.set_xlim([min(enu_x_cmd), max(enu_x_cmd)])
ax3.set_ylim([min(enu_y_cmd), max(enu_y_cmd)])
ax3.set_zlim([0, 50])

actual_pos_array = np.array([df['x'], df['y'], df['z']])
horizon_pos_array = np.array([x_traj, y_traj, z_traj])
N = len(x_traj[0])

position_data = [actual_pos_array, horizon_pos_array]
labels = ['Actual Position', 'Horizon']

lines = [ax3.plot([], [], [])[0] for _ in range(len(position_data))] 
color_list = sns.color_palette("hls", 1)

for i, line in enumerate(lines):
    line._color = color_list[i]
    #line._color = color_map[uav_list[i]]
    line._linewidth = 5.0
    line.set_label(labels[i])
    
patches = lines
TIME_SPAN = 20

def init():
    lines = [ax3.plot(uav[0, 0:1], 
                      uav[1, 0:1], 
                      uav[2, 0:1])[0] for uav in position_data]
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

ax3.legend()
line_ani = FuncAnimation(fig3, update_lines, 
                fargs=(position_data, patches), init_func=init,
                interval=0.1, blit=True, 
                repeat=True, 
                frames=position_data[0].shape[1]+1)

# plt.show()