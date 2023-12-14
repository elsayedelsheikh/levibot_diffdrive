FROM althack/ros2:humble-dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
      tmux    \
      neovim   \ 
      i2c-tools \
      libi2c-dev \
      python3-pip \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-teleop-twist-keyboard  \
      ros-${ROS_DISTRO}-hardware-interface \
      ros-${ROS_DISTRO}-controller-manager \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-urdf-tutorial \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc