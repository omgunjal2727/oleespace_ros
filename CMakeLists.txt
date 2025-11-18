cmake_minimum_required(VERSION 3.8)
# CRITICAL: This must match the <name> tag in your package.xml
project(oleespace_ros)

find_package(ament_cmake REQUIRED)
# If you have python scripts in 'scripts/' or C++ nodes, add dependencies here (e.g. rclcpp)

# ---------------------------------------------------------
# INSTALL RULES
# ---------------------------------------------------------

# 1. Install Launch Files
# This copies the entire 'launch' folder to install/share/oleespace_ros/launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Configuration Files
# This copies the 'config' folder (with your yaml files) to install/share/oleespace_ros/config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install Map Files
# This copies the 'maps' folder to install/share/oleespace_ros/maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install Scripts (Optional)
# If you want to run your scripts using 'ros2 run', install them here:
install(PROGRAMS
  scripts/launch_lidar.sh
  scripts/run_container.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()