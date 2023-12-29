#!/bin/bash

# 创建一个新的tmux会话
tmux new-session -d -s my_session

# 在第一个窗格中启动roscore
tmux send-keys -t my_session:0 "roscore" C-m
sleep 5

# 在第二个窗格中进入phy_simulator/rviz目录并启动rviz
tmux split-window -v -t my_session:0
tmux send-keys -t my_session:0 "roscd phy_simulator/rviz/" C-m
tmux send-keys -t my_session:0 "rviz -d phy_simulator_planning.rviz" C-m

# 在第三个窗格中启动test_ssc_with_eudm_ros.launch
tmux split-window -v -t my_session:0
tmux send-keys -t my_session:0 "roslaunch planning_integrated test_ssc_with_eudm_ros.launch" C-m

# 在第四个窗格中启动onlane_ai_agent.launch
tmux split-window -v -t my_session:0
tmux send-keys -t my_session:0 "roslaunch ai_agent_planner onlane_ai_agent.launch" C-m

# 在第五个窗格中启动phy_simulator_planning.launch
tmux split-window -v -t my_session:0
tmux send-keys -t my_session:0 "roslaunch phy_simulator phy_simulator_planning.launch" C-m

# 切换到第一个窗格
tmux select-pane -t my_session:0

# 进入tmux会话
tmux attach-session -t my_session

