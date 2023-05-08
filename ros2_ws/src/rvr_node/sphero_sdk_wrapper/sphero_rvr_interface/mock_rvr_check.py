import os

def should_mock_rvr() -> bool:
    return os.getenv("MOCK_RVR").lower() == 'true'