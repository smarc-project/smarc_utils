# smarc_utils
Common utils that are needed for different AUVs

## smarc_bringup

The bringup scripts are used to start tmux sessions with ROS nodes and launch files.
To install tmux, type `sudo apt-get install tmux`.

Running a bringup script will start a new tmux session
from which you can start all the nodes necessary for that particular session, including a `roscore`.
Everything in a tmux session is organized as tabs.

After starting, `ctrl+b`+`n` takes you to the next tab, while `ctrl+b`+`p` takes
you to the previous. `ctrl+b`+`d` allows you to disconnect and `tmux at` attaches again.
Step through all the tabs and start the nodes using `ENTER`. After you get to the last
node, or back to the first one, everything is launched.
For more information on tmux, see http://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/ .

As an alternative to tmux, the launch files under this folder can be used to launch an AUV, a Gazebo world and the control and navigation components required.

This is the only launch file that has to be modified in order to boot up a SMARC simulation.

## smarc_keyboard_teleop

To use this node, you need to install the `pygame` player.
Install it with `sudo apt-get install python-pygame`.

Run the node with `rosrun smarc_keyboard_teleop`.
In the resulting window, you can press `UP`, `DOWN`,`LEFT`, `RIGHT` to steer,
`w` to start the thruster, and `s` to stop it.

## odom_tf_bc

This node broadcasts the tf world --> odom (see REP105 ROS), with the odom frame being instantiated in the pose where the AUV is initialized.

An example of the current, simplified tf tree can be seen below
![alt Text](https://github.com/ignaciotb/smarc_utils/blob/working_branch/docs/images/tf_tree.pdf)

It also publishes the ground truth pose of the AUV provided by Gazebo and referred to the odom reference frame.

If there isn't an odometry provider, it will broadcast the odom --> base_link transform based on the pose in Gazebo (set in the smarc_bringup launch files).
