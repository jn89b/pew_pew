import pandas as pd
import numpy as np
import os 

cwd = os.getcwd()
print('dir', cwd)

folder_dir = cwd+ '/drone_ros/log/'
print("folder_dir", folder_dir)

#read csv file from folder_dir
df = pd.read_csv(folder_dir + 'compare_2.csv') 
df = df.dropna()

mavros_roll = np.rad2deg(df['roll'])
mavros_pitch = np.rad2deg(df['pitch'])
mavros_yaw = np.rad2deg(df['yaw'])

telem_roll = np.rad2deg(df['telem_roll'])
telem_pitch = np.rad2deg(df['telem_pitch'])
telem_yaw = np.rad2deg(df['telem_yaw'])

from matplotlib import pyplot as plt

plt.plot(df['time'], mavros_yaw, label='mavros_yaw')
plt.plot(df['time'], -telem_yaw, label='telem_yaw')
plt.legend()
plt.show()