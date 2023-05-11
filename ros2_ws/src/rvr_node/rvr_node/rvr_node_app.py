import time

# import ros2 packages
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg
import sensor_msgs.msg

from rvr_interfaces.action import ChangeHeading

# import sphero sdk packages
from sphero_sdk_wrapper.sphero_rvr_interface import SpheroRvrInterface, initialize_rvr_interface, initialize_rvr_sdk
from sphero_sdk_wrapper.sphero_sdk_raspberry_python.sphero_sdk import RvrStreamingServices

from .imu_data import ImuData

rvr_sdk = initialize_rvr_sdk()

streaming_interval_ms = 50

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
        
        self.publish_rvr_imu_sensor = self.create_publisher(sensor_msgs.msg.Imu, 'rvr_imu', 10)
        self.imu_data = ImuData()
        self.sensor_publish_timer = self.create_timer(
            0.1,
            self.sensor_publish_callback
        )

        self.rvr.add_sensor_data_handler(RvrStreamingServices.quaternion, self.quaternion_sensor_handler)
        self.rvr.add_sensor_data_handler(RvrStreamingServices.accelerometer, self.accelerometer_sensor_handler)
        self.rvr.add_sensor_data_handler(RvrStreamingServices.gyroscope, self.gyroscope_sensor_handler)
        self.rvr.start_sensor_streaming(streaming_interval_ms)

        self.get_logger().info('RvrNode init finished')

    def close(self):
        self.rvr.stop_sensor_streaming()
        self.rvr.close()

    def keep_alive(self):
        self.rvr.wake()

    def command_callback(self):
        if self.event_to_process:
            self.get_logger().info('Event to process')
            self.event_to_process = False
            if self.speed != 0:
                self.get_logger().info(f'start_roll: speed "{self.speed}", heading "{self.heading}"')
                self.rvr.start_roll(
                    speed=self.speed,
                    heading=self.heading
                )
            else:
                self.get_logger().info(f'stop_roll: heading "{self.heading}"')
                self.rvr.stop_roll(
                    heading=self.heading
                )

    def sensor_publish_callback(self):
        imu_msg = self.imu_data.get_imu_msg()
        self.get_logger().info(f'sensor pub: quaternion: [{imu_msg.orientation.w},{imu_msg.orientation.x},{imu_msg.orientation.y},{imu_msg.orientation.z}]')
        self.publish_rvr_imu_sensor.publish(imu_msg)

    def quaternion_sensor_handler(self, data: dict[str, float]):
        self.imu_data.set_quaternion(data['W'], data['X'], data['Y'], data['Z'])
    
    def accelerometer_sensor_handler(self, data: dict[str, float]):
        self.imu_data.set_linear_acceleration(data['X'], data['Y'], data['Z'])

    def gyroscope_sensor_handler(self, data: dict[str, float]):
        self.imu_data.set_angular_velocity_degrees(data['X'], data['Y'], data['Z'])

    def start_roll(self, msg: std_msgs.msg.Float32):
        self.speed = int(round(msg.data))
        self.event_to_process = True

    def stop_roll(self, msg: std_msgs.msg.Empty):
        self.speed = 0
        self.event_to_process = True

    def set_heading_local(self, new_heading: float):
        retval = abs(self.heading - new_heading)
        self.heading = int(round(new_heading)) % 360
        self.event_to_process = True
        return retval

    def change_heading(self, goal_handle):
        theta = goal_handle.request.theta
        self.get_logger().info(f'change_heading_start, theta: "{theta}"')
        result = ChangeHeading.Result()
        feedback_msg = ChangeHeading.Feedback()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('change_heading_canceled')
            return result
        result.delta = self.set_heading_local(theta)
        feedback_msg.remaining = result.delta
        goal_handle.publish_feedback(feedback_msg)
        goal_handle.succeed()
        return result

    def set_leds(self, msg: std_msgs.msg.Float32MultiArray):
        self.get_logger().info(f'set_leds: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    rvr_node = RvrNode(initialize_rvr_interface(rvr_sdk))

    try:
        rvr_node.get_logger().info('RvrNode initialized')
        executor = MultiThreadedExecutor()

        rclpy.spin(rvr_node, executor=executor)
    finally:
        rvr_node.close()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
