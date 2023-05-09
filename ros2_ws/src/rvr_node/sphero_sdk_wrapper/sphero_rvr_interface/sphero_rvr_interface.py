from abc import ABCMeta, abstractmethod

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
    def on_will_sleep_notify(self, handler, timeout: float) -> None:
        """Run an action 10s before the RVR sleeps."""
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
        pass
