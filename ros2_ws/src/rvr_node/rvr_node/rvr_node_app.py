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
        self.get_logger().info('RvrNode init started')
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
            std_msgs.msg.Empty,
            'rvr_stop_roll',
            self.stop_roll,
            10)
        self.roll_straight_sub = self.create_subscription(
            std_msgs.msg.Float32,
            'rvr_roll_straight',
            self.roll_straight,
            10)
        self.set_heading_sub = self.create_subscription(
            std_msgs.msg.Float32,
            'rvr_set_heading',
            self.set_heading,
            10)
        self.adjust_heading_sub = self.create_subscription(
            std_msgs.msg.Float32,
            'rvr_adjust_heading',
            self.adjust_heading,
            10)
        self.reset_heading_sub = self.create_subscription(
            std_msgs.msg.Empty,
            'rvr_reset_heading',
            self.reset_heading,
            10)
        
        self.get_logger().info('RvrNode init finished')

    def start_roll(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('start_roll: "%s"' % msg.data)
        speed = int(msg.data[0])
        heading = int(msg.data[1])
        self.set_heading_local(heading)
        self.roll_start_helper(speed)
        self.get_logger().info('start_roll end %5.4f' % stopwatch.duration)

    def stop_roll(self, msg=None):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('stop_roll')
        self.loop.run_until_complete(
            self.rvr.drive_control.roll_stop(
                heading=self.heading
            )
        )
        self.get_logger().info('stop_roll end %5.4f' % stopwatch.duration)

    def roll_straight(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('roll_straight: "%s"' % msg.data)
        speed = int(msg.data)
        self.roll_start_helper(speed)
        self.get_logger().info('roll_straight end %5.4f' % stopwatch.duration)

    def roll_start_helper(self, speed):
        self.loop.run_until_complete(
            self.rvr.drive_control.roll_start(
                speed=speed,
                heading=self.heading
            )
        )

    def set_heading_local(self, heading):
        self.heading = int(heading) % 360
    
    def set_heading_helper(self):
        self.loop.run_until_complete(
            self.rvr.drive_control.set_heading(
                heading=self.heading
            )
        )

    def adjust_heading(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('adjust_heading: "%s"' % msg.data)
        heading_delta = int(msg.data)
        self.set_heading_local(self.heading + heading_delta)
        self.set_heading_helper()
        self.get_logger().info('adjust_heading end %5.4f' % stopwatch.duration)

    def set_heading(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('set_heading: "%s"' % msg.data)
        heading = int(msg.data)
        self.set_heading_local(heading)
        self.set_heading_helper()
        self.get_logger().info('set_heading end %5.4f' % stopwatch.duration)
    
    def reset_heading(self, msg=None):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('reset_heading')
        self.set_heading_local(0.0)
        self.loop.run_until_complete(
            self.rvr.drive_control.reset_heading()
        )
        self.get_logger().info('reset_heading end %5.4f' % stopwatch.duration)

    def set_leds(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('set_leds: "%s"' % msg.data)
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

    rvr_node = RvrNode(rvr, loop)

    rvr_node.get_logger().info('RvrNode initialized')
        
    # Reset the robot's heading to 0.0
    rvr_node.reset_heading()

    rvr_node.get_logger().info('Rvr heading reset')

    rclpy.spin(rvr_node)

    rvr.sensor_control.clear(),
    rvr.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()