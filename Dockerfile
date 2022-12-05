FROM ros:galactic

RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-plotjuggler-ros \
        wget \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install transforms3d pick

RUN mkdir -p /ws/src
WORKDIR /ws
COPY code_prelection /ws/src/code_prelection
# probably bug in ros-galactic-tf-geometry-msgs package, need to copy additional file
RUN wget https://raw.githubusercontent.com/ros2/geometry2/ros2/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py -O /ws/src/code_prelection/code_prelection/tf2_geometry_msgs.py
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# source entrypoint setup
ENV OVERLAY_WS /ws
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["/config/menu.py"]
