#!/usr/bin/env bash
set -eo pipefail

# ROS Lyrical's generated setup.bash expands optional trace variables directly.
# The control-plane image intentionally has no ROS installation, so source it
# only when present and before enabling nounset.
if [[ -f /opt/ros/lyrical/setup.bash ]]; then
    source /opt/ros/lyrical/setup.bash
fi
set -u
exec "$@"
