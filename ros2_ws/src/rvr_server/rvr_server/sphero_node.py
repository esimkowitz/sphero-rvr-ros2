""" 
This project is owned by Altium, and is licensed under a Creative Commons Attribution 4.0 International License.
- Software can be modified, used commercially, and distributed.
- Software can be modified and used in private.
- A license and copyright notice must be included in the software.
- Software authors provide no warranty with the software and are not liable for anything.
For any question/concerns regarding the work please contact Harnoor Singh at harnoor343@gmail.com
"""
import os
import sys
import asyncio
sys.path.append(os.path.abspath('/app/sphero-sdk/sphero-sdk-raspberrypi-python')) 

from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import RvrStreamingServices
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray
from std_msgs.msg import Float32MultiArray
import time
import json

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

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('sphero_node')
        self.publisher_ = self.create_publisher(
            String,
            'sensors',  # publish to chatter channel
            10)
        ## REMOVE
        self.remove_IMU_sub = self.create_subscription(
            Int8MultiArray,
            'remove_IMU',
            self.remove_IMU,
            10)
        self.remove_accelerometer_sub = self.create_subscription(
            Int8MultiArray,
            'remove_accelerometer',
            self.remove_accelerometer,
            10)
        self.remove_color_detection_sub = self.create_subscription(
            Int8MultiArray,
            'remove_color_detection',
            self.remove_color_detection,
            10)
        self.remove_ambient_light_sub = self.create_subscription(
            Int8MultiArray,
            'remove_ambient_light',
            self.remove_ambient_light,
            10)
        self.remove_encoders_sub = self.create_subscription(
            Int8MultiArray,
            'remove_encoders',
            self.remove_encoders,
            10)
        ## ADD
        self.add_IMU = self.create_subscription(
            Int8MultiArray,
            'subscribe_IMU',
            self.subscribe_IMU,
            10)
        self.add_accelerometer = self.create_subscription(
            Int8MultiArray,
            'subscribee_accelerometer',
            self.subscribe_accelerometer,
            10)
        self.add_color_detection = self.create_subscription(
            Int8MultiArray,
            'subscribe_color_detection',
            self.subscribe_color_detection,
            10)
        self.add_ambient_light = self.create_subscription(
            Int8MultiArray,
            'subscribe_ambient_light',
            self.subscribe_IMU,
            10)
        self.add_encoders = self.create_subscription(
            Int8MultiArray,
            'subscribe_encoders',
            self.subscribe_IMU,
            10)
        self.change_delay = self.create_subscription(
            Int8MultiArray,
            'update_delay',
            self.update_delay,
            10)
        self.drive_si = self.create_subscription(
            Float32MultiArray,
            'drive_tank_si',   
            self.drive_tank_si_callback,
            10)
        self.drive_raw = self.create_subscription(
            Float32MultiArray,
            'drive_tank_raw',   
            self.drive_tank_raw_callback,
            10)
        self.update_leds = self.create_subscription(
            Float32MultiArray,
            'change_leds',   
            self.set_leds,
            10)
        self.remove_IMU_sub 
        self.remove_accelerometer_sub
        self.remove_color_detection_sub
        self.remove_ambient_light_sub
        self.remove_encoders_sub
        self.add_IMU
        self.add_accelerometer
        self.add_color_detection
        self.add_ambient_light
        self.add_encoders
        self.change_delay
        self.drive_si
        self.drive_raw
        self.update_leds

    async def set_leds(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if debug: print("heard")
        led_data = msg.data
        R = int(led_data[0])
        G = int(led_data[1])
        B = int(led_data[2])
        await rvr.set_all_leds(
            led_group=RvrLedGroups.all_lights.value,
            led_brightness_values=[color for x in range(10) for color in [R, G, B]]
        )

    async def drive_tank_raw_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if debug: print("heard")
        drive_data = msg.data
        right_drive = int(drive_data[1])
        left_drive = int(drive_data[0])
        direction = int(drive_data[2])

        if direction == 1:
            await rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,
            left_duty_cycle=left_drive,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle=right_drive # Valid duty cycle range is 0-255
            )
        elif direction == 2:
            await rvr.raw_motors(
            left_mode=RawMotorModesEnum.reverse.value,
            left_duty_cycle=left_drive,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=right_drive # Valid duty cycle range is 0-255
            )   
        elif direction == 3:
            await rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=left_drive,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.reverse.value,
            right_duty_cycle=right_drive # Valid duty cycle range is 0-255
            )  
        else:
            await rvr.raw_motors(
            left_mode=RawMotorModesEnum.forward.value,
            left_duty_cycle=left_drive,  # Valid duty cycle range is 0-255
            right_mode=RawMotorModesEnum.forward.value,
            right_duty_cycle=right_drive # Valid duty cycle range is 0-255
            )
    async def drive_tank_si_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if debug: print("heard")
        drive_data = msg.data
        right_drive = drive_data[1]
        left_drive = drive_data[0]
        if right_drive > 1.55:
            right_drive = 1.55
        elif right_drive < -1.55:
            right_drive = -1.55
        
        if left_drive > 1.55:
            left_drive = 1.55
        elif left_drive < -1.55:
            left_drive = -1.55

        await rvr.drive_tank_si_units(
            left_velocity = left_drive,  # Valid velocity values are [-1.555..1.555]
            right_velocity = right_drive
        )
    
    def send(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        if debug: self.get_logger().info('Publishing: "%s"' % msg.data)
    
    def update_delay(self, msg):
        if debug: print("heard")
        data = msg.data
        global delay
        if data[0] < 40:
            delay = 40
        else: 
            delay = data[0]
        additionalVar  = data[1]
        if debug: print('delay rated updated to :', data[0])
   
    async def remove_IMU(self, msg):
        global delay
        global imu_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.remove_sensor_data_handler(service=RvrStreamingServices.imu)
        imu_global = {}
        await rvr.sensor_control.start(interval=delay)
        
   
    async def subscribe_IMU(self, msg):
        global delay
        global imu_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu,
            handler=imu_handler
        )
        await rvr.sensor_control.start(interval=delay)

    async def remove_accelerometer(self, msg):
        global delay
        global accelerometer_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.remove_sensor_data_handler(service=RvrStreamingServices.accelerometer)
        accelerometer_global = {}
        await rvr.sensor_control.start(interval=delay)
        
   
    async def subscribe_accelerometer(self, msg):
        global delay
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.accelerometer,
        handler=accelerometer_handler
        )
        await rvr.sensor_control.start(interval=delay)
    
    async def remove_color_detection(self, msg):
        global delay
        global color_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.remove_sensor_data_handler(service=RvrStreamingServices.color_detection)
        color_global = {}
        await rvr.sensor_control.start(interval=delay)
        
    async def subscribe_color_detection(self, msg):
        global delay
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.enable_color_detection(is_enabled=True)
        if debug: print("Starting color handler")
        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=color_detected_handler
          )
        await rvr.sensor_control.start(interval=delay)

    async def remove_ambient_light(self, msg):
        global delay
        global ambient_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.remove_sensor_data_handler(service=RvrStreamingServices.ambient_light)
        ambient_global = {}
        await rvr.sensor_control.start(interval=delay)
        
    async def subscribe_ambient_light(self, msg):
        global delay
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.ambient_light,
            handler=ambient_light_handler
        )
        await rvr.sensor_control.start(interval=delay)

    async def remove_encoders(self, msg):
        global delay
        global encoder_global
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.remove_sensor_data_handler(service=RvrStreamingServices.encoders)
        encoder_global = {}
        await rvr.sensor_control.start(interval=delay)
        
    async def subscribe_encoders(self, msg):
        global delay
        data = msg.data
        await rvr.sensor_control.stop()
        await rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.encoders,
            handler=encoder_handler
        )
        await rvr.sensor_control.start(interval=delay)
# Check and send all values are collected
def checkData():
    publisher.send(
        json.dumps({
                **imu_global,
                **color_global,
                **accelerometer_global,
                **ambient_global,
                **encoder_global
        })
    )
    if debug: print("spinning")
    rclpy.spin_once(publisher, timeout_sec=0.01)
    if debug: print('publish')

def clear_buffer():
    global imu_global
    global color_global
    global accelerometer_global
    global ambient_global
    global encoder_global
    imu_global = {}
    color_global = {}
    accelerometer_global = {}
    ambient_global = {}
    encoder_global = {}

async def imu_handler(imu_data):
    if debug: print('IMU data received')
    global imu_global
    global received
    imu_global = imu_data
    received = received | (1)
    checkData()
    clear_buffer()

async def color_detected_handler(color_detected_data):
    #print('Color detection data response: ', color_detected_data)
    if debug: print('Color detection data received')
    global color_global
    global received
    color_global = color_detected_data
    received = received | (1 << 1)
    checkData()
    clear_buffer()
async def accelerometer_handler(accelerometer_data):
    if debug: print('Accelerometer data received')
    global accelerometer_global
    global received
    accelerometer_global = accelerometer_data
    received = received | (1 << 2)
    checkData()
    clear_buffer()
async def ambient_light_handler(ambient_light_data):
    if debug: print('Ambient light data received')
    global ambient_global
    global received
    ambient_global = ambient_light_data
    received = received | (1 << 3)
    checkData()
    clear_buffer()
async def encoder_handler(encoder_data):
    if debug: print('Encoder data received')
    global encoder_global
    global received
    encoder_global = encoder_data
    received = received | (1 << 4)
    checkData()
    clear_buffer()

rclpy.init(args=None)
publisher = MinimalPublisher()

async def main():
    """ This program demonstrates how to enable multiple sensors to stream."""
    await rvr.wake()
    # Give RVR time to wake up
    await asyncio.sleep(2)
    await rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    )
    await rvr.sensor_control.stop()
    if debug: print("Starting imu handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.imu,
        handler=imu_handler
    )
    await rvr.enable_color_detection(is_enabled=True)
    if debug: print("Starting color handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.color_detection,
        handler=color_detected_handler
    )
    if debug: print("Starting accelerometer handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.accelerometer,
        handler=accelerometer_handler
    )
    if debug: print("Starting ambient light handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.ambient_light,
        handler=ambient_light_handler
    )
    if debug: print("Starting encoder handler")
    await rvr.sensor_control.add_sensor_data_handler(
        service=RvrStreamingServices.encoders,
        handler=encoder_handler
    )
    if debug: print("Starting sensor control")
    global delay
    await rvr.sensor_control.start(interval=delay)

if __name__ == '__main__':
    try:
        loop = asyncio.get_event_loop()
        args = sys.argv[1:]
        rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(loop,port_id=args[0])
        ) 
        asyncio.ensure_future(
            main()
        )
        loop.run_forever()

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            asyncio.gather(
                rvr.sensor_control.clear(),
                rvr.close()
            )
        )