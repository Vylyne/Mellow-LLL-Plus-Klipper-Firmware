#!/bin/bash

EXTRA_PATH="$HOME/klipper/klippy/extras/filament_motion_watcher.py"

echo "Creating Symbolic link to klippy_extras/filament_watcher.py at $EXTRA_PATH"
ln -sf "$(pwd)/$(dirname "$0")/klippy_extras/filament_watcher.py" "$EXTRA_PATH"
