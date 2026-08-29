#!/usr/bin/env bash
set -eo pipefail

# ROS Lyrical's generated setup.bash expands optional trace variables directly.
# Source it before enabling nounset so a clean container environment is supported.
source /opt/ros/lyrical/setup.bash
set -u
exec "$@"
