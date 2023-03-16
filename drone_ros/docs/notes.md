
# Notes to myself
- Requirements:
	- Be able to specify commands to drone with user command inputs
	- Situation example:
		- Arm
		- Takeoff
		- Go to position
		- Land 
- The Situation:
	- There is **WAY TOO MUCH** abstraction, hard to add new features (premature abstraction)

### Drone Node Class
- Composes of:
- User Listener Class:
	- Listens to user commands/via JSON and whatnot
- Drone Commander Class:
	- Sends command to system via mavlink protocol  
- Has a designated call sign
- Has a designated port for mavlink communication

### DroneInfo Class
- Wrapper interface to convert Mavlink information of UAS and publish it to ROS2 ecosystem
- Publishes state information of UAS 
- Publishes heartbeat of UAS system
- Can provide parameter info of UAS 

### GS Service Node
- Provides service for user to:
  - Input commands for drone
  - It will validate the command inputs
- Provides service to DroneNode to:
  - Request the stack of commands from user 

## To Do
- Add the following features to the GS Service Function:
  - [x] Command line interface for user to:
    - [x] Specify arm
    - [x] Specify takeoff
    - [x] Specify UAS type 
    - [x] Specify Goal Location
    - [x] Specify Guidance Strategy
  - [ ] Build user stories for this and test that commands are okay 
  - Add the following features 

### Commander Class
Provide api to do the following:
- [x] Arm
- [x] Disarm
- [x] Takeoff
- [x] Switch Mode
- [x] Send basic velocity commands 
- [x] Unit test these inputs

### Future Work
- Make takeoff and land become services 
- Add package dependencies of:
  - drone_interfaces
  - mpc_ros
  - other stuff

### Notes 
- For takeoff and landing, need to wait until the desired height or landing condition is met, just like how the arm protocol works 
- 
## Tests 
