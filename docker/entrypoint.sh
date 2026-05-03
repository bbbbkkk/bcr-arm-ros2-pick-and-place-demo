#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /workspace/my_brc_arm_ws/install/setup.bash ]; then
  source /workspace/my_brc_arm_ws/install/setup.bash
fi

exec "$@"
