#!/bin/bash

SESSION=milan




tmux -2 new-session -d -s $SESSION

# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'core'
tmux new-window -t $SESSION:1 -n 'strands_sim'
tmux new-window -t $SESSION:2 -n 'nav'
tmux new-window -t $SESSION:3 -n 'dummy_scitos_battery'
tmux new-window -t $SESSION:4 -n 'exe'
tmux new-window -t $SESSION:5 -n 'controller'
tmux new-window -t $SESSION:6 -n 'task_adder'

tmux select-window -t $SESSION:0
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m 
tmux send-keys "roscore" C-m
tmux select-pane -t 1
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m 
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=/media/milan/DATA/mongo_db" 


tmux select-window -t $SESSION:1
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch strands_morse bham_cs_morse.launch"

tmux select-window -t $SESSION:2
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch battery_sim battery_sim_nav.launch"

tmux select-window -t $SESSION:3
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun battery_sim dummy_scitos_battery.py"
tmux select-pane -t 1
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch battery_sim docking.launch"
tmux select-pane -t 0

tmux select-window -t $SESSION:4
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "DISPLAY=:0 roslaunch --wait task_executor mdp-executor.launch interruptible_wait:=true combined_sort:=true"
tmux select-pane -t 1
tmux send-keys "DISPLAY=:0 rosrun task_executor task_status.py"


tmux select-window -t $SESSION:5
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun battery_sim controller.py"



tmux select-window -t $SESSION:6
tmux send-keys "source /home/milan/workspace/strands_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun randomised_task_creator random_adder.py"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
tmux setw -g mode-mouse on