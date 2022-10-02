# Containerized RO2 Node for Sphero RVR
Pyhton3 based [ROS2](https://docs.ros.org/en/foxy/#) node for interacting with [Sphero RVR](https://www.sphero.com/rvr).

Runs in Docker for easy integration and streamlined dependency management.

## Current Functionality
- Stream Sensors Data
    - Data from in total 5 sensors (IMU, accelerometer, color_detection, ambient_light, and encoders) is published on the `sensors` channel.
    - The delay of sensors stream can be updated using `update_delay` subscriber.
    - Subscription to all/some sensors can be both removed and added back using `remove_sensor_name` and `remove_sensor_name` respectively.
- Issue drive commands
    - In the current implementation you can publish drive commands using RVR's `drive_tank_raw` and `drive_tank_si` methods.
- Change LED color
    - `change_leds` subscriber can be used to change the color of all the LEDs on the RVR.

## Running the ROS2 Node
- `python3 server.py your_serial_port` to run the ROS2 node.
- eg: `python3 server.py /dev/ttyAMA1` 

## Subscribing to sensors channel
This piece of code is a snip from `test_talker.py` and will simply print the data coming in from the `sensors` publisher
```
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String,'sensors',self.sensor_callback,10)
        self.subscription

    async def sensor_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

minimal_subscriber = MinimalSubscriber()

def subscribe():
    ## subscribe to sensors
    rclpy.init(args=None)
    rclpy.spin(minimal_subscriber)
```

## Publishing commands

Note: Make sure to publish data in a Float32Array to drive and led change subscribers.

- `drive_tank_si` takes an array of length 2 where arr[0] is the speed of the left track and arr[1] is speed of the right track both in m/s range [-1.55,1.55].
- `drive_tank_raw` takes an array of length 3 where arr[0] is the speed of the left track and arr[1] is speed of the right track both in normalised range [0, 255] and arr[3] as a direction.
    - direction = 1.0 is both tracks in reverse direction.
    - direction = 2.0 is left track in reverse and right track in forward direction.
    - direction = 3.0 is right track in reverse and left track in forward direction.
    - any other value to direction would direct both tracks to forward direction.
-  `change_leds` takes an array of length 3, [R, G, B] values to LEDs.

This piece of code is a snip from `test_talker.py` and can be used to publish drive and led color change commands.

```
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
```

## Removing/Subscribing to sensors
Initially, data from all 5 sensors are being published on the `sensors` channel but this could be altered according to the requirements. `remove_sensor_name` and `subscribe_sensor_name` can be used to update the sensors stream.


Note: remove/subscribe methods take a dummy message. Make sure to publish that message in Int8MultiArray.
```
# this example is specific to IMU sensor and can be updated for other sensors.
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.remove_IMU = self.create_publisher(Int8MultiArray, 'subscribe_IMU', 10)
        self.sub_IMU = self.create_publisher(Int8MultiArray, 'remove_IMU', 10)

def remove_IMU_handler(self, data):
    msg = Int8MultiArray()
    msg.data = data
    self.remove_IMU.publish(msg)    
    
def subscribe_IMU_handler(self, data):
    msg = Int8MultiArray()
    msg.data = data
    self.sub_IMU.publish(msg)

minimal_publisher = MinimalPublisher()

def remove():
    arr = []
    minimal_publisher.remove_IMU_handler(arr)

def subscribe():
    arr = []
    minimal_publisher.subscribe_IMU_handler(arr)
```

## Credit

Thank you [@DomnikN](https://github.com/DominikN) for your very well documented project [DominikN/ros2_docker_examples](https://github.com/DominikN/ros2_docker_examples).

Thank you [@gumstix/Altium](https://github.com/gumstix) for your [RVR ROS2 Node demo](https://github.com/gumstix/PKG900000001506/tree/master/demo/Sphero%20RVR/ros2%20node).

## License

MIT &copy; Evan Simkowitz, 2022
