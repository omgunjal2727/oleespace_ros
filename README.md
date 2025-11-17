# OleeSpace ROS2 SLAM System

Complete ROS2 Humble Docker setup for YDLidar TG50 SLAM and autonomous navigation.

## Features

- **YDLidar TG50** driver fully configured
- **ROS2 Humble** in isolated Docker container
- **SLAM Toolbox** for real-time mapping
- **Complete TF tree** setup (map → odom → base_link → laser_frame)
- **Nav2** integration ready
- **Persistent maps** saved to host machine

## Prerequisites

- Ubuntu 20.04/22.04
- Docker installed
- YDLidar TG50 connected via USB (`/dev/ttyUSB0`)
- X11 for visualization (RViz2)

## Installation

### 1. Clone Repository
```bash
git clone https://github.com/YOUR_USERNAME/oleespace_ros.git
cd oleespace_ros
```

### 2. Build Docker Image
```bash
# Pull base image
docker pull ghcr.io/soham2560/humble:latest

# Build custom image (takes 10-15 minutes first time)
docker build -t oleespace_ros:latest -f docker/Dockerfile .
```

### 3. Setup X11 for GUI
```bash
xhost +local:docker
```

## Quick Start

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
  oleespace_ros:latest
```

### Inside Container - Setup Environment
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## Running SLAM

Open **3 terminals** and run each command in a separate terminal:

### Terminal 1: Launch LiDAR
```bash
docker exec -it oleespace_slam bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

### Terminal 2: Launch SLAM with TF
```bash
docker exec -it oleespace_slam bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch slam_config complete_slam_launch.py
```

### Terminal 3: Visualize in RViz
```bash
docker exec -it oleespace_slam bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2
```

**In RViz:**
1. Set Fixed Frame to `map`
2. Add → By topic → `/map` → Map
3. Add → By topic → `/scan` → LaserScan
4. Move robot around to build the map

### Save Your Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_office_map
```

Maps are saved to `./maps/` directory on your host machine.

## System Architecture
```
map (SLAM origin)
 └─ odom (odometry frame)
     └─ base_link (robot base)
         └─ laser_frame (LiDAR sensor)
```

## Configuration Files

| File | Description |
|------|-------------|
| `config/ydlidar.yaml` | YDLidar TG50 parameters |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox configuration |
| `config/complete_slam_launch.py` | Unified launch file with TF tree |
| `docker/Dockerfile` | Custom ROS2 container image |

## 🔍 Troubleshooting

### LiDAR Not Detected
```bash
# Check device
ls -l /dev/ttyUSB*

# Give permissions
sudo chmod 666 /dev/ttyUSB0
```

### SLAM Dropping Messages

- Ensure all 3 terminals are running
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify scan rate: `ros2 topic hz /scan` (should be ~7 Hz)

### RViz Not Opening
```bash
# On host
xhost +local:docker
echo $DISPLAY  # Should show :0 or :1

# Test X11
docker run --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ubuntu:22.04 xeyes
```

### Container Permission Issues

The container uses the `ros` user. If you need root:
```bash
docker exec -it -u root oleespace_slam bash
```

## 📊 Performance Specs

- **Scan Rate**: 7 Hz
- **Range**: 0.05m - 50m  
- **Resolution**: 20kHz sampling
- **SLAM Update**: 1 Hz
- **Map Resolution**: 0.05m

## Development

### Rebuild After Config Changes
```bash
# Rebuild Docker image
docker build -t oleespace_ros:latest -f docker/Dockerfile .

# Or just rebuild ROS workspace inside container
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Custom Launch Files

Add your launch files to `config/` and copy them during Docker build.

## Project Structure
```
oleespace_ros/
├── README.md
├── docker/
│   └── Dockerfile              # Custom Docker image
├── config/
│   ├── ydlidar.yaml           # LiDAR configuration
│   ├── mapper_params_online_async.yaml
│   ├── slam_launch.py
│   ├── complete_slam_launch.py # Complete TF + SLAM
│   ├── slam_package.xml
│   └── slam_CMakeLists.txt
└── maps/                       # Saved maps (auto-created)
    └── .gitkeep
```

## Contributors

- OM GUNJAL
- RISHIT DARWADE

## Resources

- [YDLidar TG50 Manual](https://www.ydlidar.com/)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Base Docker Image](https://github.com/soham2560/docker_images)

## License

MIT License
