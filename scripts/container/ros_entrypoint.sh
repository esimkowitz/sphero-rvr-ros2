#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /app/ros2_ws/install/setup.bash
if ! $MOCK_RVR; then
    python3 /app/ros2_ws/install/rvr_node/lib/python3.10/site-packages/sphero_sdk_wrapper/sphero_sdk_raspberry_python/getting_started/observer/leds/set_all_leds.py
fi

exec "$@"
