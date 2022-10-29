""" 
This project is owned by Altium, and is licensed under a Creative Commons Attribution 4.0 International License.
- Software can be modified, used commercially, and distributed.
- Software can be modified and used in private.
- A license and copyright notice must be included in the software.
- Software authors provide no warranty with the software and are not liable for anything.
For any question/concerns regarding the work please contact Harnoor Singh at harnoor343@gmail.com
"""
import os
import sys
import time
sys.path.append(os.path.abspath('/app/sphero-sdk/sphero-sdk-raspberry-python')) 

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrLedGroups
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from std_msgs.msg import Float32MultiArray
import json

debug = False

received = 0x00     # received byte - fully received at 0x1f


def main(args=None):
    """ This program demonstrates how to enable multiple sensors to stream."""
    rvr = SpheroRvrObserver()

    rvr.wake()

    rclpy.init(args=args)

    # Give RVR time to wake up
    time.sleep(2)
    rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )
    rvr.sensor_control.stop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()