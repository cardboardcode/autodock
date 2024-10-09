FROM osrf/ros:noetic-desktop-full-focal

# only copy autodock_core and autodock_examples repo
COPY autodock_core /root/catkin_ws/src/autodock_core
COPY autodock_examples /root/catkin_ws/src/autodock_examples
COPY autodock_sim /root/catkin_ws/src/autodock_sim

SHELL ["bash", "-c"]

# add install deps
RUN apt-get update && apt-get install -y --no-install-recommends \
  nano \
  ros-noetic-turtlebot3-gazebo \
  git && \
  rm -rf /var/lib/apt/lists/*

# install ros fiducial repo
WORKDIR /root/catkin_ws/src
RUN git clone -b noetic-devel https://github.com/UbiquityRobotics/fiducials.git
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git --depth 1 --single-branch
RUN git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git --depth 1 --single-branch

# Install dependencies
# Note: force return as true, as fiducial has some non python3 deps
# https://github.com/UbiquityRobotics/fiducials/issues/252
WORKDIR /root/catkin_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -yr

# Build repo
RUN . /opt/ros/noetic/setup.bash && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN sed -i '$isource "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
