#!/usr/bin/env python

import eventlet
eventlet.monkey_patch()
from threading import Thread
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit, Namespace

import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import std_msgs.msg

from rvr_interfaces.action import ChangeHeading

debug = True

script_dir = os.path.realpath(os.path.dirname(__file__))
template_dir = os.path.join(script_dir, 'templates')

class RobotControlPublisher(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.publish_rvr_change_leds = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_change_leds', 10)
        self.publish_rvr_start_roll = self.create_publisher(std_msgs.msg.Float32, 'rvr_start_roll', 10)
        self.publish_rvr_stop_roll = self.create_publisher(std_msgs.msg.Empty, 'rvr_stop_roll', 10)
        self.change_heading_client = ActionClient(self, ChangeHeading, 'change_heading')
        self.get_logger().info('node publisher initialized')

        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'secret!'
        self.app.debug = False
        self.app.threading = True
        self.my_ns = Namespace(self)     # Custom Flask Socket Namespace
        self.socketio = SocketIO(self.app, cors_allowed_origins="*") # Create Socket
        self.socketio.on_namespace(self.my_ns) # Pair the namespace
        self.process_timer = self.create_timer(.1,self.process)
        self.flask_thread = Thread(
            target=self.process, daemon=True)
        self.get_logger().info('flask server initialized')

    def server_start(self):
        self.flask_thread.start()
        self.app.run(host='0.0.0.0', port=8080)
        self.socketio.run(self.app)
        self.get_logger().info('flask server started')

    def server_cleanup(self):
        self.get_logger().info('Shutting down Flask server')
        self.app.shutdown()
        self.get_logger().info('Waiting for Flask thread to finish')
        self.flask_thread.join()
        self.get_logger().info('Flask thread finished')

    def rvr_change_leds(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.publish_rvr_change_leds.publish(msg)

    def rvr_send_speed(self, speed):
        msg = std_msgs.msg.Float32()
        msg.data = speed
        self.publish_rvr_start_roll.publish(msg)

    def rvr_change_heading(self, heading_theta):
        goal_msg = ChangeHeading.Goal()
        goal_msg.theta = heading_theta

        self.change_heading_client.wait_for_server()

        return self.change_heading_client.send_goal_async(goal_msg)

    def process(self):
        alive = True
        while alive:
            try:
                # Check that the ROS node is still alive
                self.assert_liveliness()
                # Spin the ROS node once...
                rclpy.spin_once(self,timeout_sec=0.0)
                # ...and then momentarily sleep so the other process (socket.io) can run.
                eventlet.greenthread.sleep()
            except Exception:
                alive = False

rclpy.init(args=None)
publisher = RobotControlPublisher()

@publisher.app.route('/')
def index():
    return render_template('index.html')

@publisher.app.route('/control_event', methods=['POST'])
def control_event():
    if request.method == 'POST':
        control_str = request.form['control']

        speed, heading = control_str.split(',')
        
        publisher.rvr_change_heading(float(round(float(heading))))
        publisher.rvr_send_speed(float(round(float(speed))))

    return 'OK'

def main():
    def end_process(signum=None):
        # Called on process termination.
        if signum is not None:
            SIGNAL_NAMES_DICT = dict((getattr(signal, n), n) for n in dir(
                signal) if n.startswith('SIG') and '_' not in n)
            print("signal {} received by process with PID {}".format(
                SIGNAL_NAMES_DICT[signum], os.getpid()))
        print("\n-- Terminating program --")
        publisher.server_cleanup()
        os._exit(0)

    # Assign handler for process exit
    signal.signal(signal.SIGTERM, end_process)
    signal.signal(signal.SIGINT, end_process)
    signal.signal(signal.SIGHUP, end_process)
    signal.signal(signal.SIGQUIT, end_process)

    publisher.server_start()

if __name__ == '__main__':
    main()
