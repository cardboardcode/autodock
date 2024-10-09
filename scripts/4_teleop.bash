#!/usr/bin/env bash

docker exec -it tb3_autodock_sim_c bash -c \
    "source /ros_entrypoint.sh && rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
