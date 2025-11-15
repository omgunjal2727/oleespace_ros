#!/bin/bash

echo "🚀 Starting OleeSpace ROS2 Humble container..."

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "⚠️  Warning: DISPLAY not set. RViz may not work."
    export DISPLAY=:0
fi

# Run container
docker run -it --rm \
  --name oleespace_ros_container \
  --privileged \
  --device=/dev/ttyUSB0 \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  --network host \
  --entrypoint /bin/bash \
  ghcr.io/soham2560/humble:latest

echo "✅ Container started successfully!"
