`src/buffer.c` is firmware — it gets compiled and flashed to the buffer board's own MCU (Installation Steps 0–6 below). `klippy_extras/` and `macros/` are host-side — they run as part of Klippy on your Klipper host (Pi/SBC) and only need `install.sh` (no compiling or flashing).

---

## Installation

> Done from your Klipper host (e.g. a Raspberry Pi). Commands assume `~/klipper` and `~/katapult` ([Arksine/katapult](https://github.com/Arksine/katapult)) are cloned, the Klipper build toolchain is installed, and `dfu-util` is available (`sudo apt install dfu-util`).

### Step 0 — Find your buffer & power it

Connect the buffer's USB to the host **and** give it **24 V on VIN** (the motor needs it; the firmware configures the TMC at first boot, so 24 V must be present then).

List USB serial devices to identify the board and note your **unique chip ID** (you'll reuse it in `printer.cfg`):

```bash
ls /dev/serial/by-id/
# e.g. usb-Klipper_stm32f072xb_3C003A...3720-if00   (or a Mellow / katapult id)
#                            └──────── your XXXXXX ────────┘
```

### Step 1 — Enter DFU mode (to install Katapult)

On the buffer board: **hold BOOT**, tap **RESET**, release **RESET**, then release **BOOT**. Confirm (needs `sudo`):

```bash
sudo dfu-util -l
# → Found DFU: [0483:df11] ... name="@Internal Flash  /0x08000000/064*0002Kg"
```

### Step 2 — Back up the existing firmware (recommended)

```bash
sudo dfu-util -a 0 -d 0483:df11 -U ~/lll_backup_flash.bin       -s 0x08000000:0x20000
sudo dfu-util -a 1 -d 0483:df11 -U ~/lll_backup_optionbytes.bin            # option bytes
```

### Step 3 — Build & flash Katapult (one-time bootloader)

Katapult lets you flash all **future** updates over USB without opening the case.

```bash
cd ~/katapult
make menuconfig
```

- Micro-controller Architecture: **STMicroelectronics STM32**
- Processor model: **STM32F072**
- Clock Reference: **8 MHz crystal**
- Communication interface: **USB (on PA11/PA12)**
- Application start offset: **8 KiB**
- Support bootloader entry on rapid double click of reset: **OFF** ⚠️ *(the LLL Plus has no usable reset button; leaving this ON makes the board fail to enumerate)*

```bash
make clean && make
# board still in DFU (Step 1); flash with mass-erase so no stale app remains:
sudo dfu-util -a 0 -d 0483:df11 -D out/katapult.bin -s 0x08000000:force:mass-erase:leave
```

Re-plug USB and confirm Katapult enumerates:

```bash
ls /dev/serial/by-id/
# → usb-katapult_stm32f072xb_XXXXXX-if00
```

### Step 4 — Add this firmware to Klipper

```bash
cp src/buffer.c ~/klipper/src/buffer.c
```

Add **one line** to `~/klipper/src/Makefile` (only compiled for the F072 buffer, never for your main board):

```make
src-$(CONFIG_MACH_STM32F072) += buffer.c
```

> ⚠️ This line is reverted by `git pull` on the Klipper tree — re-add it after updating Klipper. `buffer.c` itself is untracked and survives.

### Step 5 — Build Klipper

> ⚠️ `make menuconfig` overwrites `~/klipper/.config`. If you build other MCUs (your main board, toolhead…) from this same tree, **back up its config first** and restore it afterwards:
> ```bash
> cp ~/klipper/.config ~/klipper/.config.mainboard   # backup
> # ...build & flash the buffer...
> cp ~/klipper/.config.mainboard ~/klipper/.config    # restore later
> ```

```bash
cd ~/klipper
make menuconfig
```

- Micro-controller Architecture: **STMicroelectronics STM32**
- Processor model: **STM32F072**
- Bootloader offset: **8 KiB bootloader** *(must match Katapult)*
- Clock Reference: **8 MHz crystal**
- Communication interface: **USB (on PA11/PA12)**

```bash
make clean && make
```

### Step 6 — Flash Klipper over USB (via Katapult)

```bash
~/klippy-env/bin/python3 ~/katapult/scripts/flashtool.py \
  -f ~/klipper/out/klipper.bin \
  -d /dev/serial/by-id/usb-katapult_stm32f072xb_XXXXXX-if00
```

> Use the **klippy-env** Python — it has `pyserial` (the system Python usually doesn't). After flashing, the board re-enumerates as `usb-Klipper_stm32f072xb_XXXXXX-if00`.

### Step 7 — printer.cfg

```ini
[mcu LLL_PLUS]
serial: /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXXXX-if00
restart_method: command

[filament_switch_sensor lll_entrance]
switch_pin: ^!LLL_PLUS:PB7
pause_on_runout: True
```

**Choosing the runout behaviour** (pick one):

- **Simplest — most setups:** `pause_on_runout: True` and nothing else. On runout Klipper runs your normal `PAUSE`, *including the park position your `[gcode_macro PAUSE]` / `[pause_resume]` already defines* (the same one your slicer pause uses). Your park position lives in your `PAUSE` macro — `pause_on_runout` calls it, so you don't repeat it here.

- **Add a notification / lights / etc.:** keep `pause_on_runout: True` and add a `runout_gcode:` — it runs **after** the pause, so use it for *extra* actions (not another pause):
```ini
  runout_gcode:
      M118 LLL Plus: filament runout — paused
  insert_gcode:
      M118 LLL Plus: filament reinserted
```

- **Fully custom routine** (your own park macro, `M600`, unload sequence…): set `pause_on_runout: False` and put your macro in `runout_gcode:`:
```ini
  pause_on_runout: False
  runout_gcode:
      MY_RUNOUT_MACRO
```

> ⚠️ Don't put a bare `PAUSE` in `runout_gcode` while `pause_on_runout: True` — it's a double pause. And `pause_on_runout`/`PAUSE` both rely on `[pause_resume]` being in your config (standard on virtually all printers).

**No motor/stepper/TMC section** — the firmware owns the motor; the host only reads `PB7`. Then `FIRMWARE_RESTART`.

### Step 8 — Host-side integration (optional)

This step adds two Klipper `extras` modules on top of the firmware above — neither is required for basic auto-feed + runout, which works fully from Step 7 alone.

```bash
./install.sh
```

This symlinks `klippy_extras/buffer_manager.py` and `klippy_extras/filament_watcher.py` into `~/klipper/klippy/extras/`, and the example config from `macros/` into `~/printer_data/config/`. Because these are symlinks, a later `git pull` on this repo updates the running modules immediately — you only need to re-run `install.sh` if new files are added to the repo, not for ordinary updates.

#### `buffer_manager` — host-commanded load/unload

Wraps `buffer_set_mode`/`buffer_query_state` from the firmware. One `[buffer_manager]` section per buffer board:

```ini
[buffer_manager LLL_PLUS]
mcu: LLL_PLUS
feed_speed_mm_s: 20.0   # measure this against your own buffer - don't derive it
                        # from the VACTUAL/RPM constants in buffer.c, since it
                        # also depends on your pulley/wheel diameter.
```

Two gcode commands become available for macros:

```gcode
BUFFER_SET_MODE BUFFER=LLL_PLUS MODE=FORWARD   ; push filament
BUFFER_SET_MODE BUFFER=LLL_PLUS MODE=BACK      ; retract filament
BUFFER_SET_MODE BUFFER=LLL_PLUS MODE=STOP      ; hold
BUFFER_SET_MODE BUFFER=LLL_PLUS MODE=AUTO      ; hand control back to the
                                                ; firmware's hall-sensor logic
BUFFER_QUERY BUFFER=LLL_PLUS                   ; report last known sensor/motor state
```

A held `FORWARD`/`BACK`/`STOP` is kept alive automatically while your macro runs (re-sent to the MCU every 250 ms); the firmware's own `timeout` watchdog means a crashed or disconnected host can't leave the motor running. Always finish a load/unload macro with `MODE=AUTO` explicitly — don't rely on the watchdog expiring as your normal hand-back path.

#### `filament_watcher` — direction-aware jam detection

Useful with any downstream motion sensor (e.g. a Mellow Fly MDM) — not specific to this buffer board, though `buffer_manager` above is one of the things it can watch for reversal-induced slack:

```ini
[filament_watcher MDM]
motion_pin: LLL_PLUS:PA5      # or wherever your MDM/encoder motion pin lives
detection_length: 3.5         # match your encoder wheel's rated segment spacing
confirm_window: 0.5           # seconds - two detections within this window before pausing

# name:backlash_mm pairs, comma separated. Each name is a printer object
# exposing get_position()/get_direction() (or find_past_position(), like
# extruder). backlash_mm is how far that source can travel after a direction
# reversal before we start counting missed-pulse distance against
# detection_length - accounts for slack/tolerance in that source's path to
# the encoder. Measure both values on your own setup; don't guess them.
position_sources: extruder:2.0, buffer_manager LLL_PLUS:5.0

# this gcode executes on first event missed motion event after grace period, intention is for logging.
warn_gcode:
    RESPOND MSG="Suspected filament clog on T0 - watching..."

runout_gcode:
    RESPOND MSG="Suspected filament clog"
    # TOOL is optional - omit it entirely on non-toolchanger setups
    _FILAMENT_JAM_DETECTED TOOL=T0
```

`confirm_window` requires two missed-pulse detections within that window before escalating to `runout_gcode` — a single stray miss just logs a warning. Both `detection_length` and every `backlash_mm` value are physical properties of your specific filament path (tube length/stiffness, encoder wheel choice, toolchanger dwell) and need measuring on your own hardware rather than copied from this example.

#### Uninstalling the host-side modules

```bash
./uninstall.sh
```

Only removes files it created as symlinks — it won't touch a real file that happens to share the name. It doesn't remove the `[include ...]` line from your `printer.cfg`, or revert the buffer board's own firmware (that's a separate flash step, see [Updating later](#updating-later-no-case-opening) in reverse — flash back a backed-up `.bin` from Step 2 if you want the board off this firmware entirely).

---

## Updating later (no case opening)

```bash
cd ~/klipper && make clean && make          # (re-add the Makefile line if a git pull removed it)
~/klippy-env/bin/python3 ~/katapult/scripts/flashtool.py \
  -f ~/klipper/out/klipper.bin \
  -d /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXXXX-if00
# then FIRMWARE_RESTART
```

`flashtool` reboots the running app into Katapult over USB automatically — you can target the **Klipper** serial directly, no buttons.

The host-side modules (`klippy_extras/`, `macros/`) don't need reflashing — `git pull` in this repo updates them immediately since `install.sh` symlinks rather than copies. Run `FIRMWARE_RESTART` after pulling if either module's config or code changed.

---

## TMC2208 register values

Derived directly from Mellow's firmware parameters (via the `TMCStepper` library it uses), not guessed:

| Register | Value | Notes |
|---|---|---|
| GCONF | `0x000001C4` | spreadCycle, pdn_disable, mstep_reg_select (shaft bit set for FORWARD) |
| CHOPCONF | `0x12020055` | toff=5, vsense, 64 µsteps, intpol |
| IHOLD_IRUN | `0x00010F07` | IRUN=15, IHOLD=7 (≈ `rms_current(500)` @ Rsense 0.11) |
| PWMCONF | `0xC10D0024` | pwm_autoscale |
| VACTUAL | `77575` | 260 rpm, 64 µsteps |

Speed/current can be changed in `src/buffer.c` (`SPEED_RPM`, the register values) and re-flashed.

---

## Known behavior — Mainsail shows MCU load ~100% (false positive)

Once this firmware is running, Mainsail's **MCU Load** bar for the LLL Plus may read **~100 %**. **This is a false positive — the MCU is not overloaded.**

The autonomous auto-feed task bit-bangs the TMC2208 `VACTUAL` value over a software UART. Each update is a short, tightly-timed burst (~8 ms), which inflates the per-task timing **standard deviation** that Mainsail's load indicator is built on — even though the MCU is idle the vast majority of the time.

Measured over a full print (`klippy.log` / MCU stats):

| Metric | Value | Meaning |
|---|---|---|
| `mcu_awake` | ~0.03–0.06 | awake only **~3–6 %** of the time |
| `mcu_task_avg` | ~0.4 ms | tasks are short |
| `mcu_task_stddev` | ~2.5 ms | spikes from the bit-bang `VACTUAL` bursts (~8 ms) — this is what drives the "100 %" bar |
| `send_seq` vs `receive_seq` | equal | no command backlog |
| retransmits | a few bytes over a whole print | link healthy |
| "Timer too close" / shutdowns | **0** | no real overload |

**Conclusion:** the ~100 % reading is a display artifact of the bit-bang timing jitter, not an overloaded MCU. Auto-feed, buttons and runout all keep working normally — no action needed.

---

## Status / disclaimer

Validated on **one setup** (FLY LLL Buffer Plus, STM32F072CB).

**Bench-tested & working (firmware, `src/buffer.c`):**
- Auto-feed — hall sensors driving the motor (core state machine + `VACTUAL`).
- Manual **hold** of the feed / retract buttons (KEY1 / KEY2).
- Runout on `PB7` → print pause.

**Ported faithfully from Mellow's firmware but not individually exercised here** (no hardware/wiring to trigger them on this setup): mainboard signal control (`PB5`/`PB6`), the buttons' single-/double-click side-effects (EXT pin output, pause toggle), and the 60 s jam timeout.

**Host-side modules (`klippy_extras/buffer_manager.py`, `klippy_extras/filament_watcher.py`) are newer and not yet hardware-validated.** The host-commanded mode override and its watchdog fallback are implemented against the firmware commands above but haven't been exercised through a real load/unload cycle yet. The jam-detection backlash/grace logic depends on `detection_length`, `confirm_window`, and every `backlash_mm`/`feed_speed_mm_s` value being measured on real hardware rather than left at example defaults — treat the example config values as starting points, not calibrated numbers.

Pins and register values are specific to this board.

**⚠️ No warranty — use at your own risk.** This is experimental, community firmware provided **AS-IS, with NO WARRANTY of any kind** (see the [GPL-3.0](LICENSE) license — *Disclaimer of Warranty* §15 and *Limitation of Liability* §16). Flashing MCU firmware **can brick the board or damage hardware**. You assume **all** risk; the author accepts **no liability** for any damage, bricked board, failed/ruined print, or other loss. **Back up your existing firmware first** (Step 2) and make sure you understand each step before running it.

## Credits

- **[Fly3DTeam / Mellow](https://github.com/Fly3DTeam/Buffer)** — original FLY Buffer firmware & hardware (state machine ported from their open source).
- **[Nitrooxyde/Mellow-LLL-Plus-Klipper-Firmware](https://github.com/Nitrooxyde/Mellow-LLL-Plus-Klipper-Firmware)** — source for the autonomous smart-buffer auto-feed Klipper firmware this project builds on.
- **[Klipper](https://github.com/Klipper3d/klipper)** — MCU firmware framework.
- **[Katapult](https://github.com/Arksine/katapult)** — USB bootloader.
- Community Klipper integrations: [river29](https://github.com/river29/Mellow-LLLBufferPLUS-klipper), [ss1gohan13](https://github.com/ss1gohan13/BufferPLUS-klipper).

## License

[GPL-3.0](LICENSE) — derived from GPL-3.0 sources (Fly3DTeam/Buffer, Klipper).