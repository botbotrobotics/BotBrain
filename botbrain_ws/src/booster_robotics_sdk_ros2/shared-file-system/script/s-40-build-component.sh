#!/bin/sh

set -v

# closure_build_directory_component VAR_COL_CON_BUILD=0 VAR_COMPONENT_DIR_PATH="${CUSTOM_ENV_PROJECT_DIRECTORY_PATH}/booster_ros2_interface" || exit

closure_build_directory_component VAR_COL_CON_BUILD=2 VAR_COMPONENT_DIR_PATH="${CUSTOM_ENV_PROJECT_DIRECTORY_PATH}/booster_ros2_interface" VAR_COMPONENT_ROS_NAME=booster_interface || exit


closure_clean_use_less_upload_directory
closure_upload_artifact

