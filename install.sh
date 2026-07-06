#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KLIPPER_EXTRAS="$HOME/klipper/klippy/extras"
CONFIG_DIR="$HOME/printer_data/config"
KLIPPER_SRC="$HOME/klipper/src"

link_extra() {
    local filename="$1"
    local src="$SCRIPT_DIR/klippy_extras/$filename"
    local dst="$KLIPPER_EXTRAS/$filename"
    if [ ! -f "$src" ]; then
        echo "ERROR: source file not found: $src" >&2
        exit 1
    fi
    echo "Linking $filename -> $dst"
    ln -sf "$src" "$dst"
}

if [ ! -d "$KLIPPER_EXTRAS" ]; then
    echo "ERROR: $KLIPPER_EXTRAS not found - is Klipper installed at \$HOME/klipper?" >&2
    exit 1
fi
if [ ! -d "$CONFIG_DIR" ]; then
    echo "ERROR: $CONFIG_DIR not found - is Moonraker set up at \$HOME/printer_data?" >&2
    exit 1
fi

link_extra "filament_watcher.py"
link_extra "buffer_manager.py"


SRC_SRC="$SCRIPT_DIR/klippy_src/buffer.c"
SRC_DST="$KLIPPER_SRC/buffer.c"

if [ ! -f "$SRC_SRC" ]; then
	echo "ERROR: source file not found: $SRC_SRC" >&2
	exit 1
fi
if [ ! -d "$KLIPPER_SRC" ]; then
	echo "ERROR: destination directory not found: $KLIPPER_SRC" >&2
	exit 1
fi

echo "Linking src -> $SRC_DST"
ln -sf "$SRC_SRC" "$SRC_DST"

CFG_SRC="$SCRIPT_DIR/klippy_macros/filament_watcher.cfg"
CFG_DST="$CONFIG_DIR/filament_watcher.cfg"
if [ ! -f "$CFG_SRC" ]; then
    echo "ERROR: source file not found: $CFG_SRC" >&2
    exit 1
fi
echo "Linking config -> $CFG_DST"
ln -sf "$CFG_SRC" "$CFG_DST"

echo
echo "Done. Add the following to your printer.cfg, then run FIRMWARE_RESTART:"
echo "  [include filament_watcher.cfg]"