import time

from stopwatch import Stopwatch
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import std_msgs.msg

from rvr_interfaces.action import ChangeHeading
from sphero_sdk_wrapper.sphero_rvr_interface import initialize_rvr_interface

rvr = initialize_rvr_interface()

class RvrNode(Node):

    def __init__(self) -> None:
        super().__init__('rvr_node')
        self.get_logger().info('RvrNode init started')
        self.heading = 0.0
        
        self.get_logger().info('Rvr client is created, waking')
        rvr.wake()
        time.sleep(1)
        self.get_logger().info('Rvr is awake')

        rvr.on_will_sleep_notify(self.keep_alive, None)

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
        rvr.close()

    def keep_alive(self):
        rvr.wake()

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
        rvr.stop_roll(
            heading=self.heading
        )
        
        self.get_logger().info('stop_roll end %5.4f' % stopwatch.duration)

    def roll_start_helper(self, speed):
        rvr.start_roll(
            speed=speed,
            heading=self.heading
        )

    def set_heading_local(self, new_heading):
        retval = abs(self.heading - new_heading)
        self.heading = int(new_heading) % 360
        return retval
    
    def set_heading_helper(self):
        rvr.set_heading(
            heading=self.heading
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
        self.get_logger().info('set_leds end %5.4f' % stopwatch.duration)

def main(args=None):
    rclpy.init(args=args)

    rvr_node = RvrNode()

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
