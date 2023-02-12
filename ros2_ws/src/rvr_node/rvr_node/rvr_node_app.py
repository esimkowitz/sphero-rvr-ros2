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
from rclpy.action import ActionServer
import std_msgs.msg

from rvr_interfaces.action import ChangeHeading

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

    def __init__(self, loop :asyncio.AbstractEventLoop) -> None:
        super().__init__('rvr_node')
        self.get_logger().info('RvrNode init started')
        self.loop = loop
        self.rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(
                self.loop
            )
        )

        self.loop.run_until_complete(self.rvr.wake())
        self.get_logger().info('Rvr is awake')

        # Give RVR time to wake up
        self.loop.run_until_complete(asyncio.sleep(2))


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
            std_msgs.msg.Float32,
            'rvr_start_roll',   
            self.start_roll,
            10)
        self.stop_roll_sub = self.create_subscription(
            std_msgs.msg.Empty,
            'rvr_stop_roll',
            self.stop_roll,
            10)
        self._action_server = ActionServer(
            self,
            ChangeHeading,
            'change_heading',
            self.change_heading)

        self.get_logger().info('RvrNode init finished')

    def close(self):
        self.rvr.sensor_control.clear(),
        self.rvr.close()

    def start_roll(self, msg):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        self.get_logger().info('start_roll: "%s"' % msg.data)
        speed = int(msg.data)
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

    def roll_start_helper(self, speed):
        self.loop.run_until_complete(
            self.rvr.drive_control.roll_start(
                speed=speed,
                heading=self.heading
            )
        )

    def set_heading_local(self, new_heading):
        retval = abs(self.heading - new_heading)
        self.heading = int(new_heading) % 360
        return retval
    
    def set_heading_helper(self):
        self.loop.run_until_complete(
            self.rvr.drive_control.set_heading(
                heading=self.heading
            )
        )

    def change_heading(self, goal_handle):
        stopwatch = Stopwatch(3)
        stopwatch.start()
        theta = goal_handle.request.theta
        self.get_logger().info('change_heading_start, theta: "%s"' % theta)
        result = ChangeHeading.Result()
        result.delta = self.set_heading_local(theta)
        self.set_heading_helper()
        self.get_logger().info('adjust_heading end %5.4f' % stopwatch.duration)
        return result

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
    rclpy.init()

    rvr_node = RvrNode(asyncio.get_event_loop())

    rvr_node.get_logger().info('RvrNode initialized')
        
    # Reset the robot's heading to 0.0
    rvr_node.set_heading_local(0.0)
    rvr_node.set_heading_helper()

    rvr_node.get_logger().info('Rvr heading reset')

    rclpy.spin(rvr_node)

    rvr_node.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()