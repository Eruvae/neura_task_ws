ARG DOCKER_REPO="osrf/ros"
ARG ROS_DISTRO="jazzy"
ARG IMAGE_SUFFIX="-desktop-full"
FROM $DOCKER_REPO:$ROS_DISTRO$IMAGE_SUFFIX
ARG USERNAME
ARG USER_UID=2000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Install additonal packages - add any that you need
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y python3-pip python-is-python3 ssh neovim git \
    ros-$ROS_DISTRO-ros-gz ros-$ROS_DISTRO-ur-simulation-gz ros-$ROS_DISTRO-ur-moveit-config \
    ros-$ROS_DISTRO-ur-robot-driver ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-neo-nav2-bringup \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller ros-$ROS_DISTRO-rqt-tf-tree ros-$ROS_DISTRO-rviz-visual-tools
ENV SHELL /bin/bash

# Source ROS environment automatically
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Set the default user
USER $USERNAME
CMD ["/bin/bash"]
