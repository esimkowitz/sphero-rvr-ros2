import time

# import ros2 packages
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import std_msgs.msg

from rvr_interfaces.action import ChangeHeading

# import sphero sdk packages
from sphero_sdk_wrapper.sphero_rvr_interface import SpheroRvrInterface, initialize_rvr_interface, initialize_rvr_sdk

rvr_sdk = initialize_rvr_sdk()

class RvrNode(Node):

    def __init__(self, rvr_client: SpheroRvrInterface) -> None:
        super().__init__('rvr_node')
        self.get_logger().info('RvrNode init started')
        self.heading = 0
        self.speed = 0
        self.event_to_process = True

        self.rvr = rvr_client
        
        self.get_logger().info('Rvr client is created, waking')
        self.rvr.wake()
        time.sleep(1)
        self.get_logger().info('Rvr is awake')

        self.rvr.on_will_sleep_notify(self.keep_alive, None)

        self.command_timer = self.create_timer(
            0.01,
            self.command_callback)
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
        self.rvr.close()

    def keep_alive(self):
        self.rvr.wake()

    def command_callback(self):
        if self.event_to_process:
            self.get_logger().info('Event to process')
            self.event_to_process = False
            if self.speed > 0:
                self.rvr.start_roll(
                    speed=self.speed,
                    heading=self.heading
                )
            else:
                self.rvr.stop_roll(
                    heading=self.heading
                )

    def start_roll(self, msg):
        self.get_logger().info('start_roll: "%s"' % msg.data)
        self.speed = int(msg.data)
        self.event_to_process = True

    def stop_roll(self, msg=None):
        self.get_logger().info('stop_roll')
        self.speed = 0
        self.event_to_process = True

    def set_heading_local(self, new_heading):
        retval = abs(self.heading - new_heading)
        self.heading = int(new_heading) % 360
        self.event_to_process = True
        return retval

    def change_heading(self, goal_handle):
        theta = int(goal_handle.request.theta)
        self.get_logger().info('change_heading_start, theta: "%s"' % theta)
        result = ChangeHeading.Result()
        result.delta = self.set_heading_local(theta)
        return result

    def set_leds(self, msg):
        self.get_logger().info('set_leds: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    rvr_node = RvrNode(initialize_rvr_interface(rvr_sdk))

    try:
        rvr_node.get_logger().info('RvrNode initialized')

        rclpy.spin(rvr_node)
    finally:
        rvr_node.close()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
