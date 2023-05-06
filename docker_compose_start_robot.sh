echo "setting leds"
python3 sphero-sdk/sphero-sdk-raspberry-python/getting_started/observer/leds/set_all_leds.py
echo "running ros2"
ros2 launch rvr_bringup portal_and_server.launch.py