FROM ghcr.io/soham2560/humble:latest

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Switch to root for installations
USER root

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    git \
    vim \
    nano \
    wget \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user if it doesn't exist
RUN if ! id -u ros 2>/dev/null; then \
        useradd -m -s /bin/bash -G sudo ros && \
        echo "ros:ros" | chpasswd && \
        echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers; \
    fi

# Switch to ros user
USER ros

# Create workspace
WORKDIR /home/ros/ros2_ws/src

# ---------------------------------------------------------
# 1. YDLIDAR SETUP
# ---------------------------------------------------------
# Clone YDLidar packages
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git

# Build YDLidar SDK
WORKDIR /home/ros/ros2_ws/src/YDLidar-SDK
RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Install SDK (need root)
USER root
WORKDIR /home/ros/ros2_ws/src/YDLidar-SDK/build
RUN make install && \
    echo "/usr/local/lib" > /etc/ld.so.conf.d/ydlidar.conf && \
    ldconfig

# Switch back to ros user
USER ros

# ---------------------------------------------------------
# 2. COPY YOUR PACKAGE (The Major Change)
# ---------------------------------------------------------
# We copy the entire local package folder into the container workspace
# NOTE: Run docker build from OLEE_ROS_WS root
COPY --chown=ros:ros src/oleespace_ros /home/ros/ros2_ws/src/oleespace_ros

# Update YDLidar driver config with your custom yaml
# (We copy it from your package to the driver package)
RUN cp /home/ros/ros2_ws/src/oleespace_ros/config/ydlidar.yaml \
       /home/ros/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml

# ---------------------------------------------------------
# 3. BUILD
# ---------------------------------------------------------
WORKDIR /home/ros/ros2_ws

# Build the workspace (Builds both YDLidar and oleespace_ros)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-Wno-error'"

# Setup bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /home/ros/.bashrc

# Entry point
WORKDIR /home/ros
CMD ["/bin/bash"]