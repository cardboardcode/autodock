#!/usr/bin/env bash

xhost +local:docker

docker run -it --rm \
 --name tb3_autodock_sim_c \
 -e DISPLAY=$DISPLAY \
 -e TURTLEBOT3_MODEL="burger" \
 -e GAZEBO_MODEL_PATH=/home/ubuntu/catkin_ws/src/rwa3_group/models \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 --network host \
 tb3_autodock_sim:latest bash -c \
 "source /root/catkin_ws/devel/setup.bash && roslaunch autodock_sim tb3_dock_sim.launch"

