""" 
This project is owned by Altium, and is licensed under a Creative Commons Attribution 4.0 International License.
- Software can be modified, used commercially, and distributed.
- Software can be modified and used in private.
- A license and copyright notice must be included in the software.
- Software authors provide no warranty with the software and are not liable for anything.
For any question/concerns regarding the work please contact Harnoor Singh at harnoor343@gmail.com
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray
import os
import sys
import time
from threading import Thread

debug = False
drive = False
blink = False

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publish_si = self.create_publisher(Float32MultiArray, 'drive_tank_si', 10)
        self.publish_raw = self.create_publisher(Float32MultiArray, 'drive_tank_raw', 10)
        self.publish_leds = self.create_publisher(Float32MultiArray, 'change_leds', 10)

    def send_drive_si(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publish_si.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)

    def send_drive_raw(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publish_raw.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)
    
    def send_leds(self, data):
        msg = Float32MultiArray()
        msg.data = data
        self.publish_leds.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'sensors',self.sensor_callback,10)
        self.subscription  # prevent unused variable warning

    async def sensor_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    

def input_si():
    arr = [None] * 2
    arr[0] = float(input("left motor: "))
    arr[1] = float(input("right motor: "))
    return arr

def input_raw():
    arr = [None] * 3
    arr[0] = float(input("left motor: "))
    arr[1] = float(input("right motor: "))
    arr[2] = float(input("direction: 1->reverse 2->reverse/forward 3->forward/reverse else->forward"))
    return arr  

def input_leds():
    arr = [None] * 3
    arr[0] = float(input("R :"))
    arr[1] = float(input("G :"))
    arr[2] = float(input("B :"))
    return arr 

def send_leds():
    arr = input_leds()
    minimal_publisher.send_leds(arr)

def send_si():
    arr = input_si()
    minimal_publisher.send_drive_si(arr)

def send_raw():
    arr = input_raw()
    minimal_publisher.send_drive_raw(arr)

def blink_leds():
    for i in range (10):
        arr = [255, 0.0, 0.0]
        minimal_publisher.send_leds(arr)
        time.sleep(0.2)
        arr = [0.0, 255, 0.0]
        minimal_publisher.send_leds(arr)
        time.sleep(0.2)
    arr = [0.0, 0.0, 255]
    minimal_publisher.send_leds(arr)

def drive_si_steps(n, lv, rv):
    for i in range(n):
        arr = [lv, rv]
        minimal_publisher.send_drive_si(arr)

def test_drive():
    arr = [0.1, 0.3]
    minimal_publisher.send_drive_si(arr)
    time.sleep(0.1)
    arr = [-0.1, -0.3]
    minimal_publisher.send_drive_si(arr)
    time.sleep(0.1)

rclpy.init(args=None)

minimal_subscriber = MinimalSubscriber()
minimal_publisher = MinimalPublisher()

def publish():
    # Publish to commands here
    # blink led routine
    blink_leds()
    #drive tank si
    test_drive()


def subscribe():
    ## subscribe to sensors
    rclpy.spin(minimal_subscriber)

def main():
    subscribe()
    publish()

main()