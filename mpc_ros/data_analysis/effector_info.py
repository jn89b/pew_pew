import pickle
import os
import matplotlib.pyplot as plt
import numpy as np 

def get_data(folder_dir, pkl_file_name):
    
    with open(folder_dir+pkl_file_name, 'rb') as handle:
        data = pickle.load(handle)

    return data

#load pickle file from data 
curr_dir = os.getcwd()
print("Current directory: ", curr_dir)
folder_dir = curr_dir+'/data/'
pkl_name = 'front_directional_effector.pkl'

with open(folder_dir+pkl_name, 'rb') as handle:
    data = pickle.load(handle)

left_wing_pkl = 'left_wing_directional_effector.pkl'
left_wing_data = get_data(folder_dir, left_wing_pkl)

right_wing_pkl = 'right_wing_directional_effector.pkl'
right_wing_data = get_data(folder_dir, right_wing_pkl)

plt.close('all')
idx_parse = 1
idx_end = 300
uav_location = data['uav_location'][0:idx_end]
effector_position = data['effector_location'][0:idx_end]
left_wing_effector_position = left_wing_data['effector_location'][0:idx_end]
right_wing_effector_position = right_wing_data['effector_location'][0:idx_end]
# uav_wing = data['uav_wing_location'][0:25]

#combine uav_location into one array
uav_array = np.asarray(uav_location)
# uav_wing = np.asarray(uav_wing)

#3d plot of effector position 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#set equal axis

if idx_parse == 0 :
    ax.plot(effector_position[:,0], effector_position[:,1], 
        effector_position[:,2], c='r', marker='o')
    
    ax.plot(left_wing_effector_position[:,0], left_wing_effector_position[:,1], 
        left_wing_effector_position[:,2], c='r', marker='o')
    

    ax.plot(right_wing_effector_position[:,0], right_wing_effector_position[:,1],
        right_wing_effector_position[:,2], c='r', marker='o')

    ax.scatter(uav_array[0], uav_array[1], 
            uav_array[2], label='uav')
    
    # ax.scatter(uav_wing[0], uav_wing[1],
    #         uav_wing[2], label='uav_wing')  

else:
    for effect,left_effect, right_effect in zip(
        effector_position,
        left_wing_effector_position, 
        right_wing_effector_position):

        ax.plot(effect[:,0], effect[:,1], 
            effect[:,2], c='r', marker='o')

        ax.plot(left_effect[:,0], left_effect[:,1], 
            left_effect[:,2], c='g', marker='o')

        ax.plot(right_effect[:,0], right_effect[:,1], right_effect[:,2],
            c='c', marker='o')

    ax.plot(uav_array[:,0], uav_array[:,1], 
        uav_array[:,2], c='b', marker='o', label='uav')
    



    # ax.plot(uav_wing[:,0], uav_wing[:,1],
    #     uav_wing[:,2], c='g', marker='o', label='uav_wing')


ax.axis('equal')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()


#compute the distance between effector and uav
# dist = np.linalg.norm(uav_wing - uav_array, axis=1)
# print("Distance between wing and uav: ", dist)
