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
WORKDIR /home/ros
RUN mkdir -p /home/ros/ros2_ws/src

# Clone YDLidar packages
WORKDIR /home/ros/ros2_ws/src
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

# Copy configuration files
WORKDIR /home/ros/ros2_ws/src/ydlidar_ros2_driver/params
COPY --chown=ros:ros config/ydlidar.yaml ./ydlidar.yaml

# Create SLAM config package structure
WORKDIR /home/ros/ros2_ws/src
RUN mkdir -p slam_config/launch

# Copy SLAM configuration files to correct locations
COPY --chown=ros:ros config/mapper_params_online_async.yaml slam_config/
COPY --chown=ros:ros config/slam_launch.py slam_config/launch/oleespace_slam_launch.py
COPY --chown=ros:ros config/complete_slam_launch.py slam_config/launch/complete_slam_launch.py
COPY --chown=ros:ros config/slam_package.xml slam_config/package.xml
COPY --chown=ros:ros config/slam_CMakeLists.txt slam_config/CMakeLists.txt

# Make launch files executable
RUN chmod +x slam_config/launch/*.py

# Build the workspace
WORKDIR /home/ros/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-Wno-error'"

# Setup bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /home/ros/.bashrc

# Create maps directory
RUN mkdir -p /home/ros/maps

WORKDIR /home/ros

# Entry point
CMD ["/bin/bash"]