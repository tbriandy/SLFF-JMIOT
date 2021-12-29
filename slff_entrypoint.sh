#!/bin/bash
set -e

service apache2 start

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$HOME/slff/devel/setup.bash"

exec "$@"
