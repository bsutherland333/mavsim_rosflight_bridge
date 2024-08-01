# Base docker image.
ARG ROS_DISTRO
FROM osrf/ros:${ROS_DISTRO}-desktop

# Update system
RUN apt update
RUN apt upgrade -y
RUN rosdep update

# Clone ROSflight repo
WORKDIR /rosflight_ws/src
RUN git clone --recursive https://github.com/rosflight/rosflight_ros_pkgs.git

# Copy over local repo and run setup
COPY . mavsim_rosflight_bridge
WORKDIR /rosflight_ws/src/mavsim_rosflight_bridge
RUN ./setup_mavsim.sh

# Install ROS dependencies with rosdep dependencies and build packages
WORKDIR /rosflight_ws
RUN rosdep install --from-paths . --ignore-src -y
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Add source files to .bashrc for automatic sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /rosflight_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc
ENTRYPOINT bash

