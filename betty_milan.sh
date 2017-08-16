#!/bin/bash

SESSION=milan




tmux -2 new-session -d -s $SESSION
tmux set-option -g default-command "bash --rcfile /localhome/strands/milan_ws/source_ws.sh"

# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'core'
tmux new-window -t $SESSION:1 -n 'cams'
tmux new-window -t $SESSION:2 -n 'ui_nav'
tmux new-window -t $SESSION:3 -n 'exe'
tmux new-window -t $SESSION:4 -n 'battery_mgmt'
tmux new-window -t $SESSION:5 -n 'soma'
tmux new-window -t $SESSION:6 -n 'tracker'
tmux new-window -t $SESSION:7 -n 'exploration'


tmux select-window -t $SESSION:0
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "ssh bettyr" C-m
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m 
tmux send-keys "HOSTNAME=bettyr DISPLAY=:0 roslaunch strands_bringup strands_core.launch db_path:=/home/strands/milan_ws/mongo" 
tmux split-window -v
tmux select-pane -t 2
tmux send-keys "DISPLAY=:0 roslaunch --wait strands_bringup strands_robot.launch with_mux:=false with_magnetic_barrier:=false"
tmux select-pane -t 0


tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch --wait strands_bringup strands_cameras.launch head_camera:=false chest_camera:=true chest_ip:=betty chest_user:=strands"
tmux split-window -v
tmux select-pane -t 1
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch --wait  skeleton_tracker tracker.launch camera_calibration:=/home/strands/.ros/camera_info/rgb_PS1080_PrimeSense.yaml rgb_frame_id:=head_xtion_rgb_optical_frame depth_frame_id:=head_xtion_rgb_optical_frame"


tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch --wait strands_bringup strands_ui.launch mary_machine:=betty mary_machine_user:=strands"
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "DISPLAY=:0 roslaunch --wait strands_bringup strands_navigation.launch positionUpdate:=false map:=/localhome/strands/Documents/lg_march2016/cropped.yaml with_no_go_map:=true no_go_map:=/localhome/strands/Documents/lg_march2016/cropped.yaml topological_map:=lg_march2016_small topo_nav_machine:=betty chest_xtion_machine:=betty head_xtion_machine:=bettyr	"
tmux select-pane -t 0


tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch --wait task_executor mdp-executor.launch interruptible_wait:=true combined_sort:=true"
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "DISPLAY=:0 rosrun task_executor task_status.py"
tmux select-pane -t 0

tmux select-window -t $SESSION:4
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "DISPLAY=:0 rosrun battery_scheduler print_plan.py"
tmux select-pane -t 1
tmux send-keys "DISPLAY=:0 rosrun battery_scheduler battery_scheduler_node.py"


tmux select-window -t $SESSION:5
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch soma_manager soma_local.launch map_name:=lg_march2016"
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun soma_roi_manager soma_roi_node.py poisson_activity"
tmux split-window -v
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun internship_antonin action_server.py"
tmux select-pane -t 0
tmux split-window -v
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 rosrun ubd_scene_image_comparator logger.py"


tmux select-window -t $SESSION:6
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch"
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch simple_change_detector detector.launch wait_time:=30 save_mode:=true"
tmux split-window -v
tmux select-pane -t 2
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "roslaunch --wait temporal_patterns_launch temporal_patterns.launch soma_config:=poisson_activity periodic_cycle:=1440 with_activity:=false"
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 1
tmux send-keys "ssh bettyr" C-m 
tmux send-keys "source /home/strands/milan_ws/devel/setup.bash" C-m
tmux send-keys "DISPLAY=:0 roslaunch --wait vision_people_logging logging_ubd.launch"


tmux select-window -t $SESSION:7
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "rosrun edge_exploration edge_bidder.py"
tmux select-pane -t 1
tmux send-keys "rosrun randomised_task_creator random_adder.py"
tmux split-window -v
tmux select-pane -t 2
tmux send-keys "DISPLAY=:0 roslaunch --wait activity_exploration activity_exploration.launch soma_config:=poisson_activity exploration_update_interval:=600 scene_srv:=/scene_counter/scene_best_time_estimate"
tmux select-pane -t 0


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
tmux setw -g mode-mouse on
