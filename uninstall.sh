#!/bin/bash
set -euo pipefail

KLIPPER_EXTRAS="$HOME/klipper/klippy/extras"
CONFIG_DIR="$HOME/printer_data/config"
SRC_DST="$HOME/klipper/src"

unlink_if_symlink() {
    local path="$1"
    if [ -L "$path" ]; then
        echo "Removing symlink: $path"
        rm "$path"
    elif [ -e "$path" ]; then
        echo "Skipping $path - exists but is not a symlink (not touching a real file)"
    else
        echo "Skipping $path - not present"
    fi
}

unlink_if_symlink "$KLIPPER_EXTRAS/buffer_manager.py"
unlink_if_symlink "$CONFIG_DIR/macros/buffer_manager.cfg"
unlink_if_symlink "$SRC_DST/buffer.c"

echo
echo "Done. Remove this line from printer.cfg if present, then run FIRMWARE_RESTART:"
echo "  [include macros/buffer_manager.cfg]"