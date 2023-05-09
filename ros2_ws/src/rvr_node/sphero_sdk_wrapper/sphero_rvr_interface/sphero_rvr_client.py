from sphero_sdk_wrapper.sphero_sdk_raspberry_python.sphero_sdk import SpheroRvrObserver

from .sphero_rvr_interface import SpheroRvrInterface
from .utilities import should_mock_rvr

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

    def on_will_sleep_notify(self, handler, timeout: float) -> None:
        """Run an action 10s before the RVR sleeps."""
        self.rvr.on_will_sleep_notify(handler=handler, timeout=timeout)
    
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
