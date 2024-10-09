#!/usr/bin/env bash

docker exec -it tb3_autodock_sim_c bash -c \
    "source /ros_entrypoint.sh && \
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: -0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"

sleep 2

docker exec -it tb3_autodock_sim_c bash -c \
    "source /ros_entrypoint.sh && \
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
