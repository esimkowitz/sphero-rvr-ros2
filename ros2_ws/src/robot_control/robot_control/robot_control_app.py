#!/usr/bin/env python

# Impport flask and socketio packages, socketio workaround found by https://answers.ros.org/question/345286/ros2-flask-threading-issue/
import eventlet
eventlet.monkey_patch()
from threading import Thread
from flask import Flask, render_template
from flask_socketio import SocketIO, emit, Namespace

import os
import signal

# import ros2 packages
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import std_msgs.msg
import sensor_msgs.msg

from rvr_interfaces.action import ChangeHeading

debug = True

script_dir = os.path.realpath(os.path.dirname(__file__))
template_dir = os.path.join(script_dir, 'templates')

rclpy.init(args=None)
executor = MultiThreadedExecutor()

class RobotControl(Node):
    def __init__(self) -> None:
        super().__init__('robot_control_node')
        self.primary_callback_group = ReentrantCallbackGroup()
        self.publish_rvr_change_leds = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_change_leds', 10, callback_group=self.primary_callback_group)
        self.publish_rvr_start_roll = self.create_publisher(std_msgs.msg.Float32, 'rvr_start_roll', 10, callback_group=self.primary_callback_group)
        self.publish_rvr_stop_roll = self.create_publisher(std_msgs.msg.Empty, 'rvr_stop_roll', 10, callback_group=self.primary_callback_group)
        # self.change_heading_client = ActionClient(self, ChangeHeading, 'change_heading', callback_group=self.primary_callback_group)
        self.change_heading_pub = self.create_publisher(std_msgs.msg.Float32, 'rvr_change_heading', 10, callback_group=self.primary_callback_group)
        self.get_logger().info('node publishers initialized')

        self.rvr_imu_sub = self.create_subscription(
            sensor_msgs.msg.Imu,
            'rvr_imu',
            self.imu_handler,
            10,
            callback_group=self.primary_callback_group
        )
        self.rvr_imu_sub # prevent unused variable warning
        self.get_logger().info('node subscribers initialized')

        self.app = Flask(__name__)
        self.my_ns = Namespace(self)     # Custom Flask Socket Namespace
        self.socketio = SocketIO(self.app, cors_allowed_origins="*") # Create Socket
        self.socketio.on_namespace(self.my_ns) # Pair the namespace
        self.thread_continue = True
        self.flask_thread = Thread(
            target=self.process, daemon=True)
        self.get_logger().info('flask server initialized')

    def imu_handler(self, imu_msg: sensor_msgs.msg.Imu) -> None:
        self.get_logger().info(f'new imu msg: quaternion: [{imu_msg.orientation.w},{imu_msg.orientation.x},{imu_msg.orientation.y},{imu_msg.orientation.z}]')

    def server_start(self) -> None:
        self.flask_thread.start()
        self.socketio.run(self.app, host='0.0.0.0', port=8080)
        self.get_logger().info('flask server started')

    def server_cleanup(self) -> None:
        self.get_logger().info('Shutting down Flask server')
        self.app.shutdown()
        self.thread_continue = False
        self.get_logger().info('Waiting for Flask thread to finish')
        self.flask_thread.join()
        self.get_logger().info('Flask thread finished')

    def rvr_change_leds(self, data: list[float]) -> None:
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.publish_rvr_change_leds.publish(msg)

    def rvr_send_speed(self, speed: float) -> None:
        msg = std_msgs.msg.Float32()
        msg.data = speed
        self.publish_rvr_start_roll.publish(msg)

    def rvr_change_heading(self, heading_theta: float) -> None:
        # goal_msg = ChangeHeading.Goal()
        # goal_msg.theta = heading_theta

        # self.change_heading_client.wait_for_server()

        # return self.change_heading_client.send_goal_async(goal_msg)

        msg = std_msgs.msg.Float32()
        msg.data = heading_theta
        self.change_heading_pub.publish(msg)

    def process(self) -> None:
        while self.thread_continue:
            # Spin the ROS node once...
            rclpy.spin_once(self, executor=executor, timeout_sec=0.01)
            # ...and then momentarily sleep so the other process (socket.io) can run.
            eventlet.greenthread.sleep()

node = RobotControl()

@node.app.route('/')
def index():
    return render_template('index.html')

@node.socketio.on('control_event', namespace='/robot_control')
def control_event(message):
    control_str = message['control']

    speed, heading = control_str.split(',')
    
    node.rvr_change_heading(float(heading))
    node.rvr_send_speed(float(speed))

def main():
    def end_process(signum=None):
        # Called on process termination.
        if signum is not None:
            SIGNAL_NAMES_DICT = dict((getattr(signal, n), n) for n in dir(
                signal) if n.startswith('SIG') and '_' not in n)
            print("signal {} received by process with PID {}".format(
                SIGNAL_NAMES_DICT[signum], os.getpid()))
        print("\n-- Terminating program --")
        node.server_cleanup()
        os._exit(0)

    # Assign handler for process exit
    signal.signal(signal.SIGTERM, end_process)
    signal.signal(signal.SIGINT, end_process)
    signal.signal(signal.SIGHUP, end_process)
    signal.signal(signal.SIGQUIT, end_process)

    # start the server
    node.server_start()

if __name__ == '__main__':
    main()
