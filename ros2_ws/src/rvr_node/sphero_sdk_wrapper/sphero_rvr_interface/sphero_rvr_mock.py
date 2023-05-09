from . import SpheroRvrInterface

class SpheroRvrMock(metaclass=SpheroRvrInterface):
    def wake(self) -> None:
        """Wake the RVR from sleep."""
        pass

    def close(self) -> None:
        """Close the connection to the RVR and put it to sleep"""
        pass

    def on_will_sleep_notify(self, handler, timeout: int) -> None:
        """Run an action 10s before the RVR sleeps."""
        pass
    
    def start_roll(self, speed: int, heading: int) -> None:
        """Start roll at a specified speed and heading."""
        pass

    def stop_roll(self, heading: int) -> None:
        """Stop roll."""
        pass

    def set_heading(self, heading: int) -> None:
        """Set heading."""
        pass
