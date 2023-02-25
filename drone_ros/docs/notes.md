
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
- Drone Class:
	- Composes of:
		- Drone Subscriber Class
			- Subscribes to all information about anything related to UAS
		- User Listener Class:
			- Listens to user commands/via JSON and whatnot
		- Drone Commander Class:
			- Sends command to system via mavlink protocol  
	- Has a designated call sign
	- Has a designated port for mavlink communication


## To Do

### Commander Class
Provide api to do the following:
- [x] Arm
- [x] Disarm
- [x] Takeoff
- [x] Switch Mode
- [x] Send basic velocity commands 
- Should have it where user inputs the correct arguments for each method
- Unit test these inputs 


### Notes 
- For takeoff and landing, need to wait until the desired height or landing condition is met, just like how the arm protocol works 
- 
## Tests 
