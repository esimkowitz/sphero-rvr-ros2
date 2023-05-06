from abc import ABC, abstractmethod

class SpheroRvrInterface(ABC):
    @abstractmethod
    def wake(self) -> None:
        """Wake the RVR from sleep."""
        pass

    @abstractmethod
    def close(self) -> None:
        """Close the connection to the RVR and put it to sleep"""
        pass

    @abstractmethod
    def on_will_sleep_notify(self, handler, timeout: float) -> None:
        """Run an action 10s before the RVR sleeps."""
        pass
    
    @abstractmethod
    def start_roll(self, speed: float, heading: float) -> None:
        """Start roll at a specified speed and heading."""
        pass

    @abstractmethod
    def stop_roll(self, heading: float) -> None:
        """Stop roll."""
        pass

    @abstractmethod
    def set_heading(self, heading: float) -> None:
        """Set heading."""
        pass
