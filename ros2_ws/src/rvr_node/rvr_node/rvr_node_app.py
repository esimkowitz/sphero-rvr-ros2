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
from stopwatch import Stopwatch
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import RvrLedGroups
from sphero_sdk import SerialAsyncDal
import rclpy
from rclpy.node import Node
import std_msgs.msg
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

class RvrNode(Node):
    def __init__(self, rvr :SpheroRvrAsync, loop :asyncio.AbstractEventLoop) -> None:
        super().__init__('rvr_node')
        self.rvr = rvr
        self.loop = loop
        self.publisher_ = self.create_publisher(
            std_msgs.msg.String,
            'rvr_sensors',  # publish to chatter channel
            10)
        self.set_leds_sub = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'rvr_change_leds',   
            self.set_leds,
            10)
        self.start_roll_sub = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'rvr_start_roll',   
            self.start_roll,
            10)
        self.stop_roll_sub = self.create_subscription(
            std_msgs.msg.Float32,
            'rvr_stop_roll',
            self.stop_roll,
            10)
        self.set_heading_sub = self.create_subscription(
            std_msgs.msg.Float32,
            'rvr_set_heading',
            self.set_heading,
            10)
        self.reset_heading = self.create_subscription(
            std_msgs.msg.Empty,
            'rvr_reset_heading',
            self.reset_heading,
            10)

    def start_roll(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('start_roll: "%s"' % msg.data)
        speed = int(msg.data[0])
        heading = int(msg.data[1])
        self.loop.run_until_complete(
            self.rvr.drive_control.roll_start(
                speed=speed,
                heading=heading
            )
        )
        self.get_logger().info('start_roll end %5.4f' % stopwatch.duration)

    def stop_roll(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('stop_roll: "%s"' % msg.data)
        heading = int(msg.data)
        self.loop.run_until_complete(
            self.rvr.drive_control.roll_stop(
                heading=heading
            )
        )
        self.get_logger().info('stop_roll end %5.4f' % stopwatch.duration)

    def set_heading(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('set_heading: "%s"' % msg.data)
        heading = int(msg.data)
        self.loop.run_until_complete(
            self.rvr.drive_control.set_heading(
                heading=heading
            )
        )
        self.get_logger().info('set_heading end %5.4f' % stopwatch.duration)
    
    def reset_heading(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('reset_heading')
        self.loop.run_until_complete(
            self.rvr.drive_control.reset_heading()
        )
        self.get_logger().info('reset_heading end %5.4f' % stopwatch.duration)

    def set_leds(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('set_leds: "%s"' % msg.data)
        if debug: print("heard")
        led_data = msg.data
        R = int(led_data[0])
        G = int(led_data[1])
        B = int(led_data[2])
        self.loop.run_until_complete(
            self.rvr.set_all_leds(
                led_group=RvrLedGroups.all_lights.value,
                led_brightness_values=[color for x in range(10) for color in [R, G, B]]
            )
        )
        self.get_logger().info('set_leds end %5.4f' % stopwatch.duration)

def main(args=None):
    """ This program demonstrates how to enable multiple sensors to stream."""
    loop = asyncio.get_event_loop()
    rvr = SpheroRvrAsync(
        dal=SerialAsyncDal(
            loop
        )
    )

    loop.run_until_complete(rvr.wake())

    rclpy.init(args=args)

    # Give RVR time to wake up
    loop.run_until_complete(asyncio.sleep(2))
    loop.run_until_complete(rvr.drive_control.reset_heading())

    rvr_node = RvrNode(rvr, loop)

    rclpy.spin(rvr_node)

    rvr.sensor_control.clear(),
    rvr.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()