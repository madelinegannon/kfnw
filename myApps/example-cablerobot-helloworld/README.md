# CableRobot Hello World

This example provides a basic UI for cable-driven robot system using Clearpath motors, including preset routines for homing, jogging, and shutdown.

> NOTE: The default values are tuned for a space with ~2m of vertical travel.

### Homing the Motors
This system moves the robots with absolute coordinate, so you **must** home the motors every time you power them on. You only need to do this once per session (unless of course you loose power at any momemnt).

Use the `Run Homing` button to home each motor when you first startup. The GUI will be orange if a motor is not yet homed.

The homing routine will timeout within `60 seconds` — if you need more time to home a motor, just run the routing again.

> NOTE: the homing routine is also helpful for rewinding a spool.

![image](https://github.com/madelinegannon/kfnw/blob/main/myApps/example-cablerobot-helloworld/assets/cablerobot_helloworld_homing.gif)

Once all motors are homed, the `System Controller` will be ready to go.

![image](https://github.com/madelinegannon/kfnw/blob/main/myApps/example-cablerobot-helloworld/assets/cablerobot_helloworld_is_homed.gif)

### UI Features
I built in a few keyboard shortcuts in anticipation of adding a lot motors to the system. 

- Click the header of each panel to open / collapse.
- Use `+` / `-` to open / collapse all the Motor panels

![image](https://github.com/madelinegannon/kfnw/blob/main/myApps/example-cablerobot-helloworld/assets/cablerobot_helloworld_gui.gif)

### Motor UI Panel
Each Motor has a UI that lets you individually control it:

![image](https://github.com/madelinegannon/kfnw/blob/main/myApps/example-cablerobot-helloworld/assets/cablerobot_helloworld_motor_gui.PNG)

| Panel         | Description   | 
| ------------- |:--------------| 
| Status        | MotorState: `NOT_HOMED`, `HOMING`, `DISABLED`, `ENABLED`, `ESTOP` | 
| Control       | Macros for Enabling/Disabling, E-Stop, Homing, and Shutdown     | 
| Info          | _Read-only_ panel with position, velocity, and acceleration data   | 
| Limits        | Changes the velocity, acceleration, and safety bounds   |
| Jogging       | Custom speeds and distance to jog up or down   | 
| Move To       | Trigger a position move to an absolute position (in mm)   | 

### Synchronizing Motors
You can also synchronize all the motors to mimic a "lead" motor. When you synchronize, all the other motors are assigned the motion, limits, and jogging parameters of the lead motor. Then are then all moved to the same position of the lead motor.

- Select the lead motor from the `Synchronize Index`
- Press `Synchronize` to lock / unlock all motor parameters to the lead motor.

![image](https://github.com/madelinegannon/kfnw/blob/main/myApps/example-cablerobot-helloworld/assets/cablerobot_helloworld_synchronize.gif)

#### To Do
- Continue to relay all gui input from lead motor un-synchronized

