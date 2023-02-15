import numpy as np

def create_obstacles(num_obstacles=1, obstacle_diameter=0.5, 
    x_min=0, x_max=10, y_min=0, y_max=10):
    
    """
    Create 2d obstacles in the environment
    """
    obstacles = []
    for i in range(num_obstacles):

        if i == num_obstacles-1:
            x = GOAL_X
            y = GOAL_Y
            obstacles.append([x, y, obstacle_diameter])
            continue

        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        obstacles.append([x, y, obstacle_diameter])

    return obstacles


## START 
START_X = 0
START_Y = 0
START_PSI = np.deg2rad(0)

## GOAL
GOAL_X = -75
GOAL_Y = -75
GOAL_Z = 50
GOAL_PSI = 0

#### OBSTACLES ####
OBSTACLE_AVOID = False
MOVING_OBSTACLE = False
MULTIPLE_OBSTACLE_AVOID = True

OBSTACLE_X = 10
OBSTACLE_Y = 10         
OBSTACLE_DIAMETER = 20
OBSTACLE_VX = 0.0
OBSTACLE_VY = 0.0

X_MAX = 150
Y_MAX = 100
X_MIN = -150
Y_MIN = 0

N_OBSTACLES = 20 # +1 for goal
if MULTIPLE_OBSTACLE_AVOID:
    OBSTACLES = create_obstacles(N_OBSTACLES, OBSTACLE_DIAMETER,
        x_min=X_MIN, x_max=X_MAX, y_min=Y_MIN, y_max=Y_MAX) 

ROBOT_DIAMETER = 10

#NLP solver options
MAX_ITER = 250
MAX_TIME = 0.15
PRINT_LEVEL = 0
ACCEPT_TOL = 1e-6
ACCEPT_OBJ_TOL = 1e-6
PRINT_TIME = 0


#Target options
TARGET_DISCHARGE_RATE = 0.1