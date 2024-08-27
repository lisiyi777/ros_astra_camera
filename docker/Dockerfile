FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]
RUN ln -snf /usr/share/zoneinfo/America/New_York /etc/localtime && echo "/usr/share/zoneinfo/America/New_York" > /etc/timezone

ENV ROS_DISTRO=noetic

# Install dependencies
RUN apt-get update && apt-get install -y \
    libuvc-dev \
    udev\
    git \
    curl \
    wget \
    build-essential \
    gcc \
    make \
    apt-utils \
    software-properties-common \
    libgflags-dev \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-publisher \
    libusb-1.0-0-dev \
    libeigen3-dev \
    python3 \
    python3-pip \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install numpy python-pcl

RUN mkdir -p /project
WORKDIR /project

CMD ["bash"]
