# Notes to self on MPC 

## To do:

- Build unit tests to 
  - assert values assigned are correct 
  - parameters are flagged correctly
  - catch any potential errors with missing values

- Configurable launch file to define:
  - state limits of quadcopter 
  - documentation of how this code will work

- Refactor MPC code:
  - Allow user to flag option for obstacle avoidance and



## Testing in ROS2
- https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html
- https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

```
colcon test --packages-select <name-of-pkg> --pytest-args -k name_of_the_test_function
```

```
colcon test-result --all --verbose
```



## To DO Feb 16/2022
-[x] Subscribe to Telemetry Topic 
-[x] Published to Trajectory Topic 
  - Note remove list input for telem
- [x] Lateral velocity commands with Quadcopter system

## Future Tasks
-[] Refactor dictionary so user can define it in ros2 param ecosystem
-[] Refactor config.py allow user to define and specify mission (Talk to Jon on how to play this out)
-[] Need to find a sleek way to save this trajectory data - use rosbag? 

