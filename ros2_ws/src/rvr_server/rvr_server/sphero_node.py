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

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import RvrLedGroups
from sphero_sdk import SerialAsyncDal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from std_msgs.msg import Float32MultiArray
import json

debug = False
delay = 250
size = 31 

# sensor variable initialization
imu_global = {}
color_global = {}
accelerometer_global = {}
ambient_global = {}
encoder_global = {}

received = 0x00     # received byte - fully received at 0x1f

class SpheroNode(Node):
    def __init__(self, rvr :SpheroRvrAsync, loop :asyncio.AbstractEventLoop) -> None:
        super().__init__('sphero_node')
        self.rvr = rvr
        self.loop = loop
        self.publisher_ = self.create_publisher(
            String,
            'rvr_sensors',  # publish to chatter channel
            10)
        self.update_leds = self.create_subscription(
            Float32MultiArray,
            'rvr_change_leds',   
            asyncio.run_coroutine_threadsafe(self.set_leds, self.loop),
            10)

    async def set_leds(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if debug: print("heard")
        led_data = msg.data
        R = int(led_data[0])
        G = int(led_data[1])
        B = int(led_data[2])
        await self.rvr.set_all_leds(
            led_group=RvrLedGroups.all_lights.value,
            led_brightness_values=[color for x in range(10) for color in [R, G, B]]
        )


def main(args=None):
    """ This program demonstrates how to enable multiple sensors to stream."""
    loop = asyncio.new_event_loop()
    rvr = SpheroRvrAsync(
        dal=SerialAsyncDal(
            loop
        )
    )

    loop.run_until_complete(rvr.wake())

    rclpy.init(args=args)

    # Give RVR time to wake up
    loop.run_until_complete(asyncio.sleep(2))

    sphero_node = SpheroNode(rvr)

    rclpy.spin(sphero_node)

    rvr.sensor_control.clear(),
    rvr.close()

    rclpy.shutdown()

    loop.close()

if __name__ == '__main__':
    main()