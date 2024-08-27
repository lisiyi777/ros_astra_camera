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
    && rm -rf /var/lib/apt/lists/*

# Set up the ROS workspace
# RUN mkdir -p /root/ros_astra_ws/src
# WORKDIR /root/ros_astra_ws/src
# RUN git clone https://github.com/orbbec/ros_astra_camera.git

# Build the workspace
# WORKDIR /root/ros_astra_ws
# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make"

# Source the workspace on container start
# WORKDIR /root/ros_astra_ws
# RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
#     echo "source /root/ros_astra_ws/devel/setup.bash" >> /root/.bashrc

RUN mkdir -p /project
WORKDIR /project

CMD ["bash"]
