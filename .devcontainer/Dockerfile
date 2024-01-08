FROM osrf/ros:noetic-desktop-full

ARG ROS_DISTRO=noetic
ARG DEBIAN_FRONTEND=noninteractive

# update locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# install dev tools
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    wget \
    git

# install tigervncserver
RUN wget https: //sourceforge.net/projects/tigervnc/files/stable/1.12.0/ubuntu-20.04LTS/$(dpkg --print-architecture)/tigervncserver_1.12.0-1ubuntu1_$(dpkg --print-architecture).deb && \
    apt-get install -y ./tigervncserver_*.deb

# Add vscode user with same UID and GID as your host system
# (copied from https: //code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/$USERNAME/catkin_ws/src

RUN export DISPLAY=: 0


# default workspace
WORKDIR /home/$USERNAME/catkin_ws
