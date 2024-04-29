#!/bin/bash
# This script is run after the container is created
set -e

# Update container
sudo apt-get update
# sudo apt-get install -y pkg-config $HOME/lib/*.deb

# Download repos
# find ${HOME}/repos/ -type f -exec vcs import --recursive ${ROBOT_WS} --input {} \;
mkdir -p ${USER_WORKSPACE}/src/deps
vcs import --input src/safety_module/dependencies.repos.yaml --recursive ${USER_WORKSPACE}/src/deps

# Download dependencies
local_deps.sh

# Retry at least 5 times to install dependencies
counter=0
until rosdep update --rosdistro=${ROS_DISTRO} || [ $counter -eq 5 ]; do
  ((counter++))
  echo "rosdep update failed, retrying"
  sleep 1
done
if [ $counter -eq 5 ]; then
  echo "rosdep update failed, exiting"
  exit 1
fi

rosdep install --from-paths ${USER_WORKSPACE}/src --ignore-src -y -r
