# ===========================================
# FSDS ROS Noetic Docker Image
# Formula Student Driverless Simulator
# ===========================================
FROM ros:noetic-ros-base-focal

LABEL maintainer="HYCU Autonomous Driving Competition"
LABEL description="ROS Noetic + FSDS Development Environment"

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# ===========================================
# System Dependencies
# ===========================================
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    curl \
    wget \
    # Python
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-numpy \
    # ROS packages
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-rqt-multiplot \
    ros-noetic-joy \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-rviz \
    ros-noetic-rqt* \
    # FSDS dependencies
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    # Visualization (X11)
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# ===========================================
# Python Dependencies
# ===========================================
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    msgpack-rpc-python \
    airsim

# ===========================================
# ROS Workspace Setup
# ===========================================
WORKDIR /root

# Clone FSDS Repository
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git \
    --recurse-submodules --depth 1

# Build AirLib
WORKDIR /root/Formula-Student-Driverless-Simulator/AirSim
RUN ./setup.sh && ./build.sh

# Setup Catkin Workspace
WORKDIR /root/Formula-Student-Driverless-Simulator/ros
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build"

# ===========================================
# Custom Scripts Directory
# ===========================================
RUN mkdir -p /root/catkin_ws/src/fsds_scripts/scripts
WORKDIR /root/catkin_ws

# ===========================================
# Environment Setup
# ===========================================
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash" >> ~/.bashrc && \
    echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc

# Entry point
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
