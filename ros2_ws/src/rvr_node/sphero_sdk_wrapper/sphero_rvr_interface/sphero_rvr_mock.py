from . import SpheroRvrInterface

from typing import Callable

from sphero_sdk_wrapper.sphero_sdk_raspberry_python.sphero_sdk import RvrStreamingServices

import threading, time, random

class SpheroRvrMock(metaclass=SpheroRvrInterface):
    def __init__(self):
        self.sensor_handlers = {}
        self.sensor_timer_thread = threading.Thread(target=self.timer_callback, daemon=True)
        self.sensor_timer_continue = False
        self.sensor_timer_interval = 0.0
    
    def wake(self) -> None:
        """Wake the RVR from sleep."""
        pass

    def close(self) -> None:
        """Close the connection to the RVR and put it to sleep"""
        pass

    def on_will_sleep_notify(self, handler: Callable, timeout: int) -> None:
        """Run an action 10s before the RVR sleeps."""
        pass

    def add_sensor_data_handler(self, service: str, handler: Callable[any, None]) -> None:
        """Add a sensor data handler for the specified sensor service."""
        self.sensor_handlers[service] = handler
    
    def timer_callback(self):
        while self.sensor_timer_continue:
            for service, handler in self.sensor_handlers.items():
                match service:
                    case RvrStreamingServices.quaternion:
                        data = {
                            'Quaternion': 
                            {
                                'is_valid': True, 
                                'W': random.random(), 
                                'X': 0.050207495933912494, 
                                'Y': -0.03013604856797869, 
                                'Z': -0.0034241077963784727
                            }, 
                            'Accelerometer': 
                            {
                                'is_valid': True, 
                                'X': random.random(), 
                                'Y': -0.09291076289790468, 
                                'Z': 0.9573230782890043
                            }, 
                            'Gyroscope': 
                            {
                                'is_valid': True, 
                                'X': random.random(), 
                                'Y': 0.14162110168058462, 
                                'Z': 4.6566128730773926e-07
                            }
                        }
                        handler(data)
                    case RvrStreamingServices.accelerometer, RvrStreamingServices.gyroscope:
                        data = {'X': 1.0, 'Y': 1.0, 'Z': 1.0}
                        handler(data)
                    case _:
                        # Do nothing
                        pass
            time.sleep(self.sensor_timer_interval)

    def start_sensor_streaming(self, interval: int) -> None:
        """Start the sensor streaming at the specified interval."""
        self.timer_interval = float(interval) / 1000.0
        self.sensor_timer_continue = True
        self.sensor_timer_thread.start()

    def stop_sensor_streaming(self) -> None:
        """Stop all sensor streaming."""
        self.sensor_timer_continue = False
        self.sensor_timer_thread.join()
    
    def start_roll(self, speed: int, heading: int) -> None:
        """Start roll at a specified speed and heading."""
        pass

    def stop_roll(self, heading: int) -> None:
        """Stop roll."""
        pass

    def set_heading(self, heading: int) -> None:
        """Set heading."""
        pass
