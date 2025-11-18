# OleeSpace ROS2 SLAM System

Complete ROS2 Humble Docker setup for YDLidar TG50 SLAM and autonomous navigation.

## 🚀 Features

- **YDLidar TG50** driver fully configured
- **ROS2 Humble** in isolated Docker container
- **SLAM Toolbox** for real-time mapping
- **Complete TF tree** setup (map → odom → base_link → laser_frame)
- **Nav2** integration ready
- **Persistent maps** saved to host machine

## 📋 Prerequisites

- Ubuntu 20.04/22.04
- Docker installed
- YDLidar TG50 connected via USB (`/dev/ttyUSB0`)
- X11 for visualization (RViz2)

## 🔧 Installation

### 1. Clone Repository

```bash
# Create a workspace folder
mkdir -p ~/OLEE_ROS_WS/src
cd ~/OLEE_ROS_WS/src

# Clone the package
git clone https://github.com/YOUR_USERNAME/oleespace_ros.git
```

### 2. Build Docker Image

Go to the root of your workspace (OLEE_ROS_WS) to build the image.

```bash
cd ~/OLEE_ROS_WS

# Build custom image (takes 10-15 minutes first time)
# Note: Run this from the folder containing the 'src' directory
docker build -t oleespace_slam:latest -f src/oleespace_ros/docker/Dockerfile .
```

### 3. Setup X11 for GUI

```bash
xhost +local:docker
```

## 🎯 Quick Start

### Launch Container

```bash
docker run -it --rm \
  --name oleespace_slam \
  --privileged \
  --network host \
  --entrypoint /bin/bash \
  -v /dev:/dev \
  -v $(pwd)/maps:/home/ros/maps \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  oleespace_slam:latest
```

### Inside Container - Setup Environment

The container is pre-configured, but if you need to source manually:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## 🗺️ Running SLAM

Open 3 terminals and run each command in a separate terminal:

### Terminal 1: Launch LiDAR

```bash
docker exec -it oleespace_slam bash
source ~/ros2_ws/install/setup.bash

sudo chmod 666 /dev/ttyUSB0
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### Terminal 2: Launch SLAM with TF

Note: We now use the correct package name `oleespace_ros`

```bash
docker exec -it oleespace_slam bash
source ~/ros2_ws/install/setup.bash

ros2 launch oleespace_ros complete_slam_launch.py
```

### Terminal 3: Visualize in RViz

```bash
docker exec -it oleespace_slam bash
source ~/ros2_ws/install/setup.bash

rviz2
```

In RViz:
- Set Fixed Frame to `map`
- Add → By topic → /map → Map
- Add → By topic → /scan → LaserScan
- Move robot around to build the map

### Save Your Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_office_map
```

Maps are saved to `./maps/` directory on your host machine (synced via Docker volume).

## 📐 System Architecture

```
map (SLAM origin)
 └─ odom (odometry frame)
     └─ base_link (robot base)
         └─ laser_frame (LiDAR sensor)
```

## ⚙️ Configuration Files

| File Location | Description |
|---------------|-------------|
| `config/ydlidar.yaml` | YDLidar TG50 parameters |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox configuration |
| `launch/complete_slam_launch.py` | Unified launch file with TF tree |
| `launch/slam_launch.py` | Base SLAM launch logic |
| `package.xml` | Package dependencies and definitions |
| `CMakeLists.txt` | Build rules for colcon |

## 🔍 Troubleshooting

### LiDAR Not Detected

```bash
# Check device
ls -l /dev/ttyUSB*

# Give permissions
sudo chmod 666 /dev/ttyUSB0
```

### Package Not Found

If you see `Package 'oleespace_ros' not found`:
- Ensure you are inside the container.
- Run: `source ~/ros2_ws/install/setup.bash`
- Run: `colcon build` (inside container) if you modified files.

### RViz Not Opening

```bash
# On host
xhost +local:docker
echo $DISPLAY  # Should show :0 or :1
```

## 📁 Project Structure

```
OLEE_ROS_WS/
└── src/
    └── oleespace_ros/          <-- Package Root
        ├── package.xml         <-- Defines dependencies
        ├── CMakeLists.txt      <-- Build configuration
        ├── launch/             <-- Launch scripts (.py)
        │   ├── complete_slam_launch.py
        │   └── slam_launch.py
        ├── config/             <-- Parameters (.yaml)
        │   ├── mapper_params_online_async.yaml
        │   └── ydlidar.yaml
        ├── maps/               <-- Map files (.pgm/.yaml)
        ├── scripts/            <-- Helper scripts (.sh)
        └── docker/             <-- Docker configuration
            └── Dockerfile
```

## 🤝 Contributors

- OM GUNJAL
- RISHIT DARWADE