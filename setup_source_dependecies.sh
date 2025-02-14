#!/usr/bin/env bash
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

set -e  
set -o pipefail

WS_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && cd ../../ && pwd)"
SRC_DIR=$WS_DIR/src

# List of dependencies (repository URL + optional branch)
DEPENDENCIES=(
    "https://github.com/PRBonn/kiss-icp.git main"
    # Add other dependencies here
)

clone_dependencies() {
    cd $SRC_DIR

    for dep in "${DEPENDENCIES[@]}"; do
        REPO_URL=$(echo "$dep" | awk '{print $1}')
        BRANCH=$(echo "$dep" | awk '{print $2}')
        FOLDER_NAME=$(basename "$REPO_URL" .git)

        if [ ! -d "$FOLDER_NAME" ]; then
            echo "Cloning $FOLDER_NAME from $REPO_URL (branch: $BRANCH)..."
            git clone --branch "$BRANCH" "$REPO_URL"
        else
            echo "$FOLDER_NAME already exists, skipping..."
        fi
    done
}

install_dependencies() {
    cd $WS_DIR
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
}

build_workspace_deps() {
    cd $WS_DIR
    echo "Building workspace with colcon..."
    colcon build --symlink-install --packages-skip antrobot_ros
}


source_workspace() {
    echo "Sourcing the workspace dependecies..."
    source $WS_DIR/install/local_setup.bash
}

main() {
    echo "Setting antrobot_ros source dependcies workspace..."
    clone_dependencies
    install_dependencies
    build_workspace_deps
    source_workspace
    echo "Setup complete!"
}

main
