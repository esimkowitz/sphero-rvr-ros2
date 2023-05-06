#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /app/ros2_ws/install/setup.bash

python3 /app/sphero-sdk/sphero-sdk-raspberry-python/getting_started/observer/leds/set_all_leds.py

exec "$@"