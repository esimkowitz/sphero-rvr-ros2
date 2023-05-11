from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from threading import Lock
from math import radians

class ImuData:
    """Thread-safe store of relevant IMU data, formatted for the sensor_msgs interface."""
    def __init__(self):
        self.lock = Lock()
        self.quaternion = Quaternion()
        self.linear_acceleration = Vector3()
        self.angular_velocity = Vector3()

    def set_quaternion(self, w: float, x: float, y: float, z: float) -> None:
        self.lock.acquire()
        self.quaternion.w = w
        self.quaternion.x = x
        self.quaternion.y = y
        self.quaternion.z = z
        self.lock.release()

    def set_linear_acceleration(self, x: float, y: float, z: float) -> None:
        self.lock.acquire()
        self.linear_acceleration.x = x
        self.linear_acceleration.y = y
        self.linear_acceleration.z = z
        self.lock.release()

    def set_angular_velocity_degrees(self, x: float, y: float, z: float) -> None:
        self.set_angular_velocity_radians(radians(x), radians(y), radians(z))
    
    def set_angular_velocity_radians(self, x: float, y: float, z: float) -> None:
        self.lock.acquire()
        self.angular_velocity.x = x
        self.angular_velocity.y = y
        self.angular_velocity.z = z
        self.lock.release()

    def get_imu_msg(self) -> Imu:
        self.lock.acquire()
        retval = Imu(orientation=self.quaternion, angular_velocity=self.angular_velocity, linear_acceleration=self.linear_acceleration)
        self.lock.release()
        return retval
        