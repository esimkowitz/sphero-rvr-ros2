#!/usr/bin/env python

from threading import Thread
from flask import Flask, render_template, request
from wsgiref.simple_server import make_server

import os
import signal

import rclpy
from rclpy.node import Node
import std_msgs.msg

app = Flask(__name__, template_folder='/app/ros2_ws/src/robot_control/robot_control/templates', static_folder='/app/ros2_ws/src/robot_control/robot_control/static')
app.config['SECRET_KEY'] = 'secret!'
app.debug = False
app.threading = True

debug = True

class RobotControlPublisher(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.publish_rvr_change_leds = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_change_leds', 10)
        self.publish_rvr_roll_straight = self.create_publisher(std_msgs.msg.Float32, 'rvr_roll_straight', 10)
        self.publish_rvr_stop_roll = self.create_publisher(std_msgs.msg.Empty, 'rvr_stop_roll', 10)
        self.publish_rvr_adjust_heading = self.create_publisher(std_msgs.msg.Float32, 'rvr_adjust_heading', 10)
        self.publish_rvr_set_heading = self.create_publisher(std_msgs.msg.Float32, 'rvr_set_heading', 10)
        self.publish_rvr_reset_heading = self.create_publisher(std_msgs.msg.Empty, 'rvr_reset_heading', 10)

    def rvr_change_leds(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.publish_rvr_change_leds.publish(msg)

    def rvr_start_roll_forward(self):
        msg = std_msgs.msg.Float32()
        msg.data = 30.0
        self.publish_rvr_roll_straight.publish(msg)

    def rvr_start_roll_reverse(self):
        msg = std_msgs.msg.Float32()
        msg.data = -30.0
        self.publish_rvr_roll_straight.publish(msg)

    def rvr_stop_roll(self):
        msg = std_msgs.msg.Empty()
        self.publish_rvr_stop_roll.publish(msg)

    def rvr_adjust_heading(self, heading_delta):
        msg = std_msgs.msg.Float32()
        msg.data = heading_delta
        self.publish_rvr_adjust_heading.publish(msg)

    def rvr_set_heading(self, heading):
        msg = std_msgs.msg.Float32()
        msg.data = heading % 360.0
        self.publish_rvr_set_heading.publish(msg)

    def rvr_turn_left(self):
        self.rvr_adjust_heading(-30.0)

    def rvr_turn_right(self):
        self.rvr_adjust_heading(30.0)

    def rvr_reset_heading(self):
        msg = std_msgs.msg.Empty()
        self.publish_rvr_reset_heading.publish(msg)

rclpy.init(args=None)
publisher = RobotControlPublisher()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control_event', methods=['POST'])
def control_event():
    if request.method == 'POST':
        control = request.form['control']

        match control:
            case 'f': 
                publisher.rvr_start_roll_forward()
                publisher.get_logger().info('robot forward')
            case 'b':
                publisher.rvr_start_roll_reverse()
                publisher.get_logger().info('robot backward')
            case 'l':
                publisher.rvr_turn_left()
                publisher.get_logger().info('robot left')
            case 'r':
                publisher.rvr_turn_right()
                publisher.get_logger().info('robot right')
            case 's':
                publisher.rvr_stop_roll()
                publisher.get_logger().info('robot stop')
            case _:
                publisher.get_logger().warn('unknown command to control_event')

    return 'OK'

class Server():
    def __init__(self):
        self.flask_server = make_server(
            '', 8080,
            app=app)
        self.flask_thread = Thread(
            target=self.flask_server.serve_forever)
        publisher.get_logger().info('flask server initialized')
        self.flask_thread.start()
        publisher.get_logger().info('flask server started')

    def cleanup(self):
        publisher.get_logger().info('Shutting down Flask server')
        self.flask_server.shutdown()
        publisher.get_logger().info('Waiting for Flask thread to finish')
        self.flask_thread.join()
        rclpy.shutdown()

def main():
    def end_process(signum=None):
        # Called on process termination.
        if signum is not None:
            SIGNAL_NAMES_DICT = dict((getattr(signal, n), n) for n in dir(
                signal) if n.startswith('SIG') and '_' not in n)
            print("signal {} received by process with PID {}".format(
                SIGNAL_NAMES_DICT[signum], os.getpid()))
        print("\n-- Terminating program --")
        server.cleanup()
        os._exit(0)

    # Assign handler for process exit
    signal.signal(signal.SIGTERM, end_process)
    signal.signal(signal.SIGINT, end_process)
    signal.signal(signal.SIGHUP, end_process)
    signal.signal(signal.SIGQUIT, end_process)

    server = Server()

if __name__ == '__main__':
    main()
