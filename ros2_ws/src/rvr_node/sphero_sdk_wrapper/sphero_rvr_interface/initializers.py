from sphero_sdk_wrapper.sphero_sdk_raspberry_python.sphero_sdk import SpheroRvrObserver

from .sphero_rvr_interface import SpheroRvrInterface
from .sphero_rvr_client import SpheroRvrClient
from .sphero_rvr_mock import SpheroRvrMock
from .utilities import should_mock_rvr

def initialize_rvr_sdk() -> SpheroRvrObserver:
    return None if should_mock_rvr() else SpheroRvrObserver()

def initialize_rvr_interface(rvr: SpheroRvrObserver) -> SpheroRvrInterface:
    return SpheroRvrMock() if should_mock_rvr() else SpheroRvrClient(rvr)
