# OleeSpace ROS

Complete ROS2 Humble setup for YDLidar TG50 SLAM and localization in Docker.

## 🚀 Features

- YDLidar TG50 driver configured and ready
- ROS2 Humble in Docker container
- SLAM Toolbox for mapping and localization
- RViz2 for visualization
- Easy setup scripts for quick deployment

## 📋 Prerequisites

- Docker installed on Ubuntu
- YDLidar TG50 connected via USB
- VS Code (optional but recommended)

## ⚡ Quick Start

### 1. Clone this repository
```bash
git clone https://github.com/YOUR_USERNAME/oleespace_ros.git
cd oleespace_ros
```

### 2. Allow X11 for GUI (RViz)
```bash
xhost +local:docker
```

### 3. Run the container
```bash
chmod +x scripts/run_container.sh
./scripts/run_container.sh
```

### 4. Inside container, build workspace (first time only)
```bash
chmod +x /workspace/scripts/setup_workspace.sh
/workspace/scripts/setup_workspace.sh
```

### 5. Launch LiDAR with RViz
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```

## 🗺️ SLAM & Mapping

### Start SLAM for mapping
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

### Save the map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Use saved map for localization
```bash
ros2 launch slam_toolbox localization_launch.py map:=~/maps/my_map.yaml
```

## ⚙️ Configuration

YDLidar TG50 parameters in `config/ydlidar.yaml`:
- **Baudrate**: 512000 bps
- **Lidar Type**: 0 (ToF)
- **Frequency**: 7Hz
- **Sample Rate**: 20kHz
- **Range**: 0.05m - 50m

## 🔧 Troubleshooting

### Lidar not detected
```bash
# Check USB device
ls -l /dev/ttyUSB*

# Give permissions
sudo chmod 666 /dev/ttyUSB0
```

### RViz not opening
```bash
# On host machine
xhost +local:docker
echo $DISPLAY  # Should show :0 or :1
```

### Container stops unexpectedly
```bash
# Check lidar connection
docker logs ros2_humble_lidar

# Restart with verbose output
docker exec -it ros2_humble_lidar bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

## 📁 Project Structure
```
oleespace_ros/
├── README.md
├── config/
│   └── ydlidar.yaml          # YDLidar TG50 configuration
├── scripts/
│   ├── run_container.sh      # Docker container launcher
│   └── setup_workspace.sh    # Workspace build script
└── docker/
    └── Dockerfile            # Custom Docker image (optional)
```

## 🤝 Team

- Add your team members here

## 📚 Resources

- [YDLidar TG50 Datasheet](https://www.ydlidar.com/products/view/6.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

## 📝 License

MIT License - Feel free to use for your projects!

---

**Built with ❤️ for robotics and SLAM**
