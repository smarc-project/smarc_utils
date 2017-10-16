SESSION=pipe_following
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'rviz'
tmux new-window -t $SESSION:2 -n 'gazebo'
tmux new-window -t $SESSION:3 -n 'small_smarc_auv'
tmux new-window -t $SESSION:4 -n 'keyboard_teleop'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rosrun rviz rviz"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch underwater_envs pipe_following.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch small_smarc_auv upload_small_smarc_auv.launch x:=-253.7 y:=23.4 z:=-92.5 pitch:=0.38"

tmux select-window -t $SESSION:4
tmux send-keys "rosrun smarc_keyboard_teleop teleop.py"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
