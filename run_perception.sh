#!/bin/bash

# Check if user has Docker permissions
if ! docker info >/dev/null 2>&1; then
    echo "Error: User does not have permission to access Docker daemon. Try adding user to 'docker' group with:"
    echo "  sudo usermod -aG docker $USER"
    echo "Then log out and back in, or run 'newgrp docker'. Alternatively, run this script with sudo."
    exit 1
fi

# Check if interfaces package is built on host, if not, build it
if [ ! -d "install/interfaces" ]; then
    echo "Interfaces package not found. Building the workspace on host..."
    colcon build --packages-select interfaces
    if [ $? -ne 0 ]; then
        echo "Failed to build the interfaces package. Exiting."
        exit 1
    fi
fi

# Function to check if a Docker image exists
check_docker_image() {
    local image_name="$1"
    if ! docker image inspect "$image_name" >/dev/null 2>&1; then
        echo "Docker image $image_name not found."
        return 1
    fi
    echo "Docker image $image_name found."
    return 0
}

# Check and build perception image if necessary
PERCEPTION_IMAGE="probe_perception:latest"
if ! check_docker_image "$PERCEPTION_IMAGE"; then
    echo "Building perception image $PERCEPTION_IMAGE..."
    docker build -t "$PERCEPTION_IMAGE" -f docker/dockerfile_perception .
    if [ $? -ne 0 ]; then
        echo "Failed to build perception image $PERCEPTION_IMAGE. Exiting."
        exit 1
    fi
fi

# Source the ROS 2 workspace on host
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start docker compose in the foreground
cd docker
docker compose up

# Store the exit status of docker compose
DOCKER_EXIT_STATUS=$?

# Check if docker compose exited successfully
if [ $DOCKER_EXIT_STATUS -ne 0 ]; then
    echo "Docker Compose failed with exit status $DOCKER_EXIT_STATUS. Exiting."
    exit $DOCKER_EXIT_STATUS
fi