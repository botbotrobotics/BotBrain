#!/bin/sh

set -v

case "${CUSTOM_ENV_OS_LINUX_VERSION}" in

*"22"*)
  export CUSTOM_ENV_ROS_SUFFIX="humble"
  closure_deb_download_install VAR_DEB_NAME="ros2-apt-source-22"
  ;;

*"24"*)
  export CUSTOM_ENV_ROS_SUFFIX="kilted"
  closure_deb_download_install VAR_DEB_NAME="ros2-apt-source-24"
  ;;

esac

closure_apt_get_install_package VAR_PACKAGE_NAME="ros-${CUSTOM_ENV_ROS_SUFFIX}-ament-cmake"
closure_apt_get_install_package VAR_PACKAGE_NAME="ros-${CUSTOM_ENV_ROS_SUFFIX}-rclcpp"
closure_apt_get_install_package VAR_PACKAGE_NAME="ros-${CUSTOM_ENV_ROS_SUFFIX}-rosidl-default-generators"

pip install colcon-common-extensions==0.3.0 || exit
pip install empy==3.3.4 || exit
pip install numpy==1.26.4 || exit
pip install lark==1.2.2 || exit

# closure_apt_get_install_package VAR_PACKAGE_NAME="python3-empy"
# closure_apt_get_install_package VAR_PACKAGE_NAME="python3-colcon-common-extensions"
closure_apt_get_remove_package VAR_PACKAGE_NAME="gfortran-12"

. /opt/ros/"${CUSTOM_ENV_ROS_SUFFIX}"/setup.sh
. "${CUSTOM_ENV_PROJECT_DIRECTORY_PATH}"/shared-file-system/script/s-40-build-component.sh
