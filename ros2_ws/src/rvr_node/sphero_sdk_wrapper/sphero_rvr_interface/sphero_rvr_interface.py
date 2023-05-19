from abc import ABCMeta, abstractmethod
from typing import Callable
from ctypes import c_uint8

class SpheroRvrInterface(ABCMeta):
    @abstractmethod
    def wake(self) -> None:
        """Wake the RVR from sleep."""
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        """Close the connection to the RVR and put it to sleep"""
        raise NotImplementedError

    @abstractmethod
    def on_will_sleep_notify(self, handler: Callable, timeout: float) -> None:
        """Run an action 10s before the RVR sleeps."""
        raise NotImplementedError
    
    @abstractmethod
    def add_sensor_data_handler(self, service: str, handler: Callable[[dict[str, float]], None]) -> None:
        """Add a sensor data handler for the specified sensor service."""
        raise NotImplementedError
    
    @abstractmethod
    def start_sensor_streaming(self, interval: int) -> None:
        """Start the sensor streaming at the specified interval (in ms)."""
        raise NotImplementedError
    
    @abstractmethod
    def stop_sensor_streaming(self) -> None:
        """Stop all sensor streaming."""
        raise NotImplementedError

    @abstractmethod
    def start_roll(self, speed: int, heading: int) -> None:
        """Start roll at a specified speed and heading."""
        raise NotImplementedError

    @abstractmethod
    def stop_roll(self, heading: int) -> None:
        """Stop roll."""
        raise NotImplementedError

    @abstractmethod
    def set_heading(self, heading: int) -> None:
        """Set heading."""
        raise NotImplementedError

    @abstractmethod
    def drive_with_heading(self, speed: int, heading: int, timeout : float | None = None) -> None:
        """Start driving at a set heading and speed."""
        raise NotImplementedError
