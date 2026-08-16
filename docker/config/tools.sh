# tools.sh

# Image
IMAGE_NAME="ros-jazzy"
IMAGE_TAG="ros-jazzy"
IMAGE_RASP_TAG="arm64"
DOCKERFILE="docker/robot.dockerfile"

# Container
CONTAINER_NAME="ros_container"
USERNAME="host"

# Workspace (host -> container)
ROS_WS="ros_ws"
WORKSPACE_HOST="$(pwd)/${ROS_WS}"
WORKSPACE_CONTAINER="/home/${USERNAME}/${ROS_WS}"