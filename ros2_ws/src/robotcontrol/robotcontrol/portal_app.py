#!/usr/bin/env python

# Robot controller, modified from the example provided in the Flask-SocketIO
# GitHub repo.
# Modified by Evan Simkowitz (esimkowitz@wustl.edu), July 2017

# Does not work, need to clean up bits from old setup, get logic working so that it can send messages over the ROS publisher


import argparse
from flask import Flask, render_template, session, request, Response

import sys
import io
import os
import shutil
import signal
import time
import urllib

from subprocess import Popen, PIPE, check_output
from string import Template
from struct import Struct
from threading import Thread
from time import sleep, time

import rclpy
from rclpy.node import Node
import std_msgs.msg
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.debug = False
app.threading = True

debug = False

class RosPublisher(Node):
    def __init__(self):
        super().__init__('robotcontrol_publisher')
        self.rvr_change_leds = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_change_leds', 10)
        self.rvr_start_roll = self.create_publisher(std_msgs.msg.Float32MultiArray, 'rvr_start_roll', 10)
        self.rvr_stop_roll = self.create_publisher(std_msgs.msg.Float32, 'rvr_stop_roll', 10)
        self.rvr_set_heading = self.create_publisher(std_msgs.msg.Float32, 'rvr_set_heading', 10)
        self.rvr_reset_heading = self.create_publisher(std_msgs.msg.Empty, 'rvr_reset_heading', 10)

    def rvr_change_leds(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.rvr_change_leds.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def rvr_start_roll(self, data):
        msg = std_msgs.msg.Float32MultiArray()
        msg.data = data
        self.rvr_start_roll.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def rvr_stop_roll(self, data):
        msg = std_msgs.msg.Float32()
        msg.data = data
        self.rvr_stop_roll.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def rvr_set_heading(self, data):
        msg = std_msgs.msg.Float32()
        msg.data = data
        self.rvr_set_heading.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def rvr_reset_heading(self, data):
        msg = std_msgs.msg.Empty()
        msg.data = data
        self.rvr_reset_heading.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/control_event', methods=['POST'])
def control_event():
    if request.method == 'POST':
        control = request.form['control']
        if control == 'f':
            try:
                robot.forward(75)
            except:
                if is_robot:
                    raise RuntimeError("Unknown error with robot")
                else:
                    pass
            print("robot forward")
        elif control == 'b':
            try:
                robot.backward(75)
            except:
                if is_robot:
                    raise RuntimeError("Unknown error with robot")
                else:
                    pass
            print("robot backward")
        elif control == 'l':
            try:
                robot.left(75)
            except:
                if is_robot:
                    raise RuntimeError("Unknown error with robot")
                else:
                    pass
            print("robot left")
        elif control == 'r':
            try:
                robot.right(75)
            except:
                if is_robot:
                    raise RuntimeError("Unknown error with robot")
                else:
                    pass
            print("robot right")
        elif control == 's':
            try:
                robot.stop()
            except:
                if is_robot:
                    raise RuntimeError("Unknown error with robot")
                else:
                    pass
            print("robot stop")
    return 'OK'


def main():
    server = Server()

    def endProcess(signum=None, frame=None):
        # Called on process termination.
        if signum is not None:
            SIGNAL_NAMES_DICT = dict((getattr(signal, n), n) for n in dir(
                signal) if n.startswith('SIG') and '_' not in n)
            print("signal {} received by process with PID {}".format(
                SIGNAL_NAMES_DICT[signum], os.getpid()))
        print("\n-- Terminating program --")
        print("Cleaning up Server...")
        server.cleanup()
        robot.stop()
        print("Done.")
        os._exit(0)

    # Assign handler for process exit
    signal.signal(signal.SIGTERM, endProcess)
    signal.signal(signal.SIGINT, endProcess)
    signal.signal(signal.SIGHUP, endProcess)
    signal.signal(signal.SIGQUIT, endProcess)

    server.start()


if __name__ == '__main__':
    main()
