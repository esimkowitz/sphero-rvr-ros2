from typing import Callable

from sphero_sdk_wrapper.sphero_sdk_raspberry_python.sphero_sdk import SpheroRvrObserver

from .sphero_rvr_interface import SpheroRvrInterface

import numpy as np

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
flag_empty = np.byte(0x00)
flag_reverse = np.byte(0x01)

class SpheroRvrClient(metaclass=SpheroRvrInterface):
    def __init__(self, rvr: SpheroRvrObserver) -> None:
        self.rvr = rvr

    def wake(self) -> None:
        """Wake the RVR from sleep."""
        self.rvr.wake()

    def close(self) -> None:
        """Close the connection to the RVR and put it to sleep"""
        self.rvr.sensor_control.clear(),
        self.rvr.close()

    def on_will_sleep_notify(self, handler: Callable, timeout: float) -> None:
        """Run an action 10s before the RVR sleeps."""
        self.rvr.on_will_sleep_notify(handler=handler, timeout=timeout)

    def add_sensor_data_handler(self, service: str, handler: Callable[[dict[str, float]], None]) -> None:
        """Add a sensor data handler for the specified sensor service."""
        self.rvr.sensor_control.add_sensor_data_handler(service, handler)
    
    def start_sensor_streaming(self, interval: int) -> None:
        """Start the sensor streaming at the specified interval (in ms)."""
        self.rvr.sensor_control.start(interval)
    
    def stop_sensor_streaming(self) -> None:
        """Stop all sensor streaming."""
        self.rvr.sensor_control.stop()
    
    def start_roll(self, speed: int, heading: int) -> None:
        """Start roll at a specified speed and heading."""
        self.rvr.drive_control.roll_start(
            speed=speed,
            heading=heading
        )

    def stop_roll(self, heading: int) -> None:
        """Stop roll."""
        self.rvr.drive_control.roll_stop(
            heading=heading
        )

    def set_heading(self, heading: int) -> None:
        """Set heading."""
        self.rvr.drive_control.set_heading(
            heading=heading
        )

    def drive_with_heading(self, speed: int, heading: int, timeout : float | None = None) -> None:
        """Start driving at a set heading and speed."""
        flags = flag_empty
        if (speed > 0): 
            flags = flag_reverse # drive reverse
            speed = abs(speed)
        
        speed_uint = np.uint8(speed)
        heading_uint = np.uint16(heading)

        self.rvr.drive_with_heading(speed_uint, heading_uint, flags, timeout)
        