# FSDS ROS Noetic Docker Image
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# System Dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git curl wget \
    python3-pip python3-catkin-tools python3-rosdep \
    python3-rosinstall python3-rosinstall-generator python3-wstool \
    ros-noetic-tf2-geometry-msgs ros-noetic-cv-bridge \
    ros-noetic-image-transport ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-laser-geometry ros-noetic-pcl-ros \
    libyaml-cpp-dev libcurl4-openssl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Python Dependencies - msgpack-rpc-python MUST be installed first (airsim dependency)
RUN pip3 install --no-cache-dir msgpack-rpc-python==0.4.1 && \
    pip3 install --no-cache-dir \
    numpy==1.24.3 scipy==1.10.1 matplotlib==3.7.2 \
    opencv-python==4.8.0.74 pytest==8.3.5 && \
    pip3 install --no-cache-dir airsim==1.8.1 fsds

# ROS Workspace
WORKDIR /root
ARG FSDS_COMMIT=v2.2.0
RUN git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git \
    --recurse-submodules --depth 1 --branch ${FSDS_COMMIT}

# Build FSDS ROS packages
WORKDIR /root/Formula-Student-Driverless-Simulator/ros
RUN rm -rf src/joy src/joystick 2>/dev/null || true && \
    mkdir -p /root/Formula-Student-Driverless-Simulator/AirSim/AirLib/deps/eigen3 && \
    ln -sf /usr/include/eigen3/Eigen /root/Formula-Student-Driverless-Simulator/AirSim/AirLib/deps/eigen3/Eigen && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release"

# Custom workspace
WORKDIR /root/catkin_ws/src
RUN ln -s /root/Formula-Student-Driverless-Simulator/ros/src/fs_msgs .

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /root/Formula-Student-Driverless-Simulator/ros/devel/setup.bash && \
    catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build fs_msgs"

# Autonomous driving code directory
RUN mkdir -p /root/catkin_ws/src/autonomous/{driver,modules,config,tests}
WORKDIR /root/catkin_ws

# Environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export PYTHONPATH=/root/catkin_ws/src/autonomous:\${PYTHONPATH}" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
