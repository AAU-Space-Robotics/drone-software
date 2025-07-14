#!/bin/bash

# Get the absolute path of the script's directory
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Set the ROS 2 workspace path and src directory
ROS_WORKSPACE_PATH="$SCRIPT_DIR"  # Base workspace is drone-software
SRC_DIR="$ROS_WORKSPACE_PATH/src"
PX4_MSGS_DIR="$SRC_DIR/px4_msgs"

# Check if src directory exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Error: src directory does not exist at $SRC_DIR"
    exit 1
fi

# Navigate to px4_msgs directory
cd "$PX4_MSGS_DIR" || {
    echo "Error: px4_msgs directory not found at $PX4_MSGS_DIR"
    exit 1
}

# Get current branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo "Current branch of px4_msgs: $CURRENT_BRANCH"

# Ask if the branch is correct
read -p "Is the branch '$CURRENT_BRANCH' correct? (y/n): " ANSWER

if [ "$ANSWER" = "n" ] || [ "$ANSWER" = "N" ]; then
    # Determine target branch
    if [ "$CURRENT_BRANCH" = "main" ]; then
        TARGET_BRANCH="release/1.15"
    else
        TARGET_BRANCH="main"
    fi

    echo "Switching to branch $TARGET_BRANCH..."
    git checkout "$TARGET_BRANCH" || {
        echo "Error: Failed to switch to branch $TARGET_BRANCH"
        exit 1
    }
    echo "Switched to branch $TARGET_BRANCH"
else
    echo "Keeping current branch $CURRENT_BRANCH"
fi

# Navigate back to workspace root
cd "$ROS_WORKSPACE_PATH" || {
    echo "Error: Failed to navigate to workspace root $ROS_WORKSPACE_PATH"
    exit 1
}

# Build message definitions
echo "Building message definitions..."
colcon build --packages-select px4_msgs || {
    echo "Error: Failed to build message definitions"
    exit 1
}

echo "Message definitions built successfully!"