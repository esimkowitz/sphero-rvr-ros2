#!/bin/bash

# This script guides a user through setting up the Sphero RVR Python SDK.

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# Install the SDK dependencies (only for the current user)
pip3 install --user -r $SCRIPT_DIR/sphero-sdk-raspberry-python/requirements.txt

# Reload ~/.profile.  In recent Raspberry Pi OS releases, this automatically includes $HOME/.local/bin in the path once it exists.
source ~/.profile