#!/bin/bash
ROOT_PATH=$(dirname "$BASH_SOURCE")
ASSETS_PATH="$ROOT_PATH/assets/"
export ROS2_5G_ERA_ASSETS_PATH=$ASSETS_PATH
echo "ROS2_5G_ERA_ASSETS_PATH set to $ASSETS_PATH"