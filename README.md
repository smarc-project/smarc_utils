# smarc_utils
Common utils that are needed for different AUVs

## smarc_bringup

The bringup scripts are used to start tmux sessions with ros nodes and launch files.
To install tmux, type `sudo apt-get install tmux`.

Running a bringup script will start a new tmux session
from which you can start all the nodes necessary for that particular session, including a `roscore`.
Everything in a tmux session is organized as tabs.

After starting, `ctrl+b`+`n` takes you to the next tab, while `ctrl+b`+`p` takes
you to the previous. `ctrl+b`+`d` allows you to disconnect and `tmux at` attaches again.
Step through all the tabs and start the nodes using `ENTER`. After you get to the last
node, or back to the first one, everything is launched.

For more information on tmux, see http://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/ .

## smarc_keyboard_teleop

To use this node, you need to install the `pygame` player.
Install it with `sudo apt-get install python-pygame`.

Run the node with `rosrun smarc_keyboard_teleop`.
In the resulting window, you can press `UP`, `DOWN`,`LEFT`, `RIGHT` to steer,
`w` to start the thruster, and `s` to stop it.
