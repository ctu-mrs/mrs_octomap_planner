#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

unattended=0
for param in "$@"
do
  echo $param
  if [[ $param == "--unattended" ]]; then
    echo "installing in unattended mode"
    unattended=1
    subinstall_params="--unattended"
  fi
done

echo "$0: Installing dependencies for MRS Octomap Planning"

# Install dependencies
echo "Installing dependencies"
sudo apt-get -y install ros-noetic-dynamic-edt-3d

echo "$0: Done"
