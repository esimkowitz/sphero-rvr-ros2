#!/usr/bin/env python

# Robot controller, modified from the example provided in the Flask-SocketIO
# GitHub repo.
# Modified by Evan Simkowitz (esimkowitz@wustl.edu), July 2017

# Does not work, need to clean up bits from old setup, get logic working so that it can send messages over the ROS publisher

from threading import Thread
from flask import Flask, render_template, request
from wsgiref.simple_server import make_server

import os
import signal

import rclpy
from rclpy.node import Node
import std_msgs.msg

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.debug = False
app.threading = True

debug = False

class RosPublisher(Node):
    def __init__(self):
        super().__init__('portal_app')
        self.rvr_change_leds = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_change_leds', 10)
        self.rvr_start_roll = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_start_roll', 10)
        self.rvr_stop_roll = self.create_publisher(std_msgs.msg.Float32, 'rvr_stop_roll', 10)
        self.rvr_set_heading = self.create_publisher(std_msgs.msg.Float32, 'rvr_set_heading', 10)
        self.rvr_reset_heading = self.create_publisher(std_msgs.msg.Empty, 'rvr_reset_heading', 10)

    def rvr_change_leds(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.rvr_change_leds.publish(msg)
        self.log_debug(msg)

    def rvr_start_roll(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.rvr_start_roll.publish(msg)
        self.log_debug(msg)

    def rvr_stop_roll(self, data):
        msg = std_msgs.msg.Float32()
        msg.data = data
        self.rvr_stop_roll.publish(msg)
        self.log_debug(msg)

    def rvr_set_heading(self, data):
        msg = std_msgs.msg.Float32()
        msg.data = data
        self.rvr_set_heading.publish(msg)
        self.log_debug(msg)

    def rvr_reset_heading(self, data):
        msg = std_msgs.msg.Empty()
        msg.data = data
        self.rvr_reset_heading.publish(msg)
        self.log_debug(msg)
    
    def log_debug(self, msg):
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/control_event', methods=['POST'])
def control_event():
    if request.method == 'POST':
        control = request.form['control']
        if control == 'f':
            server.ros_publisher.rvr_start_roll([30.0, 0.0])
            server.ros_publisher.get_logger().info('robot forward')
        elif control == 'b':
            server.ros_publisher.rvr_start_roll([30.0, 180.0])
            server.ros_publisher.get_logger().info('robot backward')
        elif control == 'l':
            server.ros_publisher.rvr_start_roll([30.0, 270.0])
            server.ros_publisher.get_logger().info('robot left')
        elif control == 'r':
            server.ros_publisher.rvr_start_roll([30.0, 90.0])
            server.ros_publisher.get_logger().info('robot right')
        elif control == 's':
            server.ros_publisher.rvr_stop_roll(0.0)
            server.ros_publisher.get_logger().info('robot stop')
    return 'OK'

class Server():
    def __init__(self, args):
        rclpy.init(args=args)
        self.flask_server = make_server(
            '', 8080,
            app=app)
        self.flask_thread = Thread(
            target=self.flask_server.serve_forever)

        self.ros_publisher = RosPublisher()

    def cleanup(self):
        print('Shutting down Flask server')
        self.flask_server.shutdown()
        print('Waiting for Flask thread to finish')
        self.flask_thread.join()
        rclpy.shutdown()

def main(args=None):
    def endProcess(signum=None, frame=None):
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
    signal.signal(signal.SIGTERM, endProcess)
    signal.signal(signal.SIGINT, endProcess)
    signal.signal(signal.SIGHUP, endProcess)
    signal.signal(signal.SIGQUIT, endProcess)

    global server 
    server = Server(args)

if __name__ == '__main__':
    main()
