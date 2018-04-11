# SMARC Simulation user's manual
Some steps to run and contribute to the SMARC simulator

## Launch the system as it is
So far, the top level launch files are located in the smarc_bringup pkg under smarc_utils.
These will call the specific control/planning/perception/navigation/utils components specified from the rest of repos, together with the desired Gazebo environment and AUV model.

### 1. Launch Gazebo with the desired scenario
The `auv_scenarios.launch` will launch the specified world from the existing ones in `smarc_worlds/launch/`

Example:
```
roslaunch smarc_bringup auv_scenarios.launch gazebo_env:=pipe_following
```

You could run RVIZ from here to visualize the environment

### 2. Launch the AUV and its system components
The `auv_system.launch` file has three flags so far:
- `namespace`: name of the AUV model. So far only lolo_auv is available.
- `number`: instance of the AUV to be launched. As many as desired from 0 onwards without repeating names.
- `navigation_on`: run the navigation stack or use the ground truth pose provided by Gazebo.

And also the initial pose of the AUV as input

Example:
```
roslaunch smarc_bringup auv_system.launch namespace:=lolo_auv number:=1 navigation_on:=true
```

## Adding new components to the system
`auv_system.launch` calls several other launch files, one (or more) per repository.
To add a new component say, for control, just go to auv_controllers pkg and add or modify the launch file there to call your control nodes in the repository, making sure the auv_system.launch calls the aimed one.
