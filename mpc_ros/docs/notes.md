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
- Subscribe to Telemetry Topic 
-    
