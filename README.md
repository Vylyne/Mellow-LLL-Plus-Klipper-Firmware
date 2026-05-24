# Mellow FLY LLL Buffer Plus — Klipper MCU Firmware (autonomous auto-feed)

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
![MCU](https://img.shields.io/badge/MCU-STM32F072CB-green)
![Klipper](https://img.shields.io/badge/Klipper-MCU%20firmware-orange)

Custom **Klipper MCU firmware** for the Mellow **FLY LLL Buffer Plus** that brings together the **best of both worlds**:

- 🟢 the **autonomous smart-buffer auto-feed** of Mellow's standalone firmware (hall sensors drive the feed motor *in firmware*, real-time), **and**
- 🟢 the board running as a **standard Klipper USB MCU**, so the filament-entrance sensor can trigger a **print pause** through Klipper.

Both at the same time, over a single USB cable (+ 24 V motor power) — something neither the stock standalone firmware nor the community host-macro configs can do on their own.

---

## Why this exists

The official Mellow firmware is **standalone**: the buffer auto-feeds on its own but the board does **not** talk Klipper (no USB MCU, runout has to go through a separate signal wire).

The community Klipper integrations ([river29](https://github.com/river29/Mellow-LLLBufferPLUS-klipper), [ss1gohan13](https://github.com/ss1gohan13/BufferPLUS-klipper)) flash **stock Klipper** and re-implement the buffer with **host-side g-code macros**. That works, but:

- feed bursts go through the **g-code queue** → latency, and they can lag behind during long `G1 E` moves;
- they switch the active extruder (`ACTIVATE_EXTRUDER`) which can **misdirect extrusion during a print** if not carefully guarded.

This project takes the other route: the buffer state machine runs **inside the MCU firmware** as a Klipper task — zero g-code-queue latency, no active-extruder juggling, and it can't stall a print. The host only reads the entrance sensor on `PB7`.

| | Standalone (Mellow) | Stock Klipper + host macros | **This firmware** |
|---|---|---|---|
| Auto-feed | ✅ in-firmware | ⚠️ host macros (queue latency) | ✅ **in-firmware** |
| Klipper MCU / USB sensor | ❌ | ✅ | ✅ |
| Risk of disrupting a print | n/a | ⚠️ `ACTIVATE_EXTRUDER` | ✅ none |
| Runout → print pause | signal wire | USB (PB7) | USB (PB7) |

---

## How it works

A small Klipper MCU task (`src/buffer.c`, `DECL_INIT` + `DECL_TASK`) ports the state machine of Mellow's open firmware ([Fly3DTeam/Buffer](https://github.com/Fly3DTeam/Buffer)) and drives the on-board TMC2208 over its single-wire UART (`VACTUAL` velocity mode):

```
HALL3 (PB4) blocked  → motor FORWARD (push filament)
HALL2 (PB3) blocked  → motor STOP
HALL1 (PB2) blocked  → motor BACK (retract)
ENDSTOP_3 (PB7) = no filament → STOP   (+ host pauses the print)
```

Photo-sensor convention is taken **verbatim** from the source: *blocked = 1, clear = 0*. Manual **feed/retract buttons** (KEY1/KEY2) and the **mainboard control signals** (PB5/PB6, active-low) are supported too. A 60 s continuous-feed timeout acts as a jam safety.

> The only change versus the standalone is that the button "hold" is handled **event-driven** (non-blocking) instead of the original blocking `while()` loop, because blocking for seconds would break Klipper host comms. Behaviour is identical.

### Pins (STM32F072CB)

| Function | Pin | Function | Pin |
|---|---|---|---|
| HALL1 | PB2 | KEY1 (retract) | PB13 |
| HALL2 | PB3 | KEY2 (feed) | PB12 |
| HALL3 | PB4 | FRONT signal | PB5 |
| Entrance switch | PB7 | BACK signal | PB6 |
| TMC EN | PA6 | TMC UART | PB1 |

---

## Installation

### 1. Add the firmware file

Copy `src/buffer.c` into your Klipper tree:

```bash
cp src/buffer.c ~/klipper/src/buffer.c
```

Add **one line** to `~/klipper/src/Makefile` (only compiles it for the F072 buffer, never for your main board):

```make
src-$(CONFIG_MACH_STM32F072) += buffer.c
```

> ⚠️ This line is reverted by `git pull` on the Klipper tree — re-add it after updating Klipper. `buffer.c` itself is untracked and survives.

### 2. Build Klipper

`make menuconfig`:

- Micro-controller: **STMicroelectronics STM32**
- Processor model: **STM32F072**
- Bootloader offset: **8 KiB** (to match Katapult)
- Clock Reference: **8 MHz crystal**
- Communication interface: **USB (on PA11/PA12)**

```bash
make clean && make
```

### 3. Install Katapult (one-time, recommended)

Lets you flash all future updates **over USB**, without opening the case. Build Katapult for F072 / 8 MHz / USB PA11-PA12 / **8 KiB offset** and **without** double-reset entry (the LLL Plus has no convenient reset button), then flash it once via DFU with a mass-erase:

```bash
sudo dfu-util -a 0 -d 0483:df11 -D ~/katapult/out/katapult.bin \
  -s 0x08000000:force:mass-erase:leave
```

### 4. Flash the firmware (over USB via Katapult)

```bash
~/klippy-env/bin/python3 ~/katapult/scripts/flashtool.py \
  -f ~/klipper/out/klipper.bin \
  -d /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXXXX-if00
```

> Use the **klippy-env** Python — it has `pyserial` (the system Python usually doesn't).

### 5. printer.cfg

```ini
[mcu LLL_PLUS]
serial: /dev/serial/by-id/usb-Klipper_stm32f072xb_XXXXXX-if00
restart_method: command

[filament_switch_sensor lll_entrance]
switch_pin: ^!LLL_PLUS:PB7
pause_on_runout: True
runout_gcode:
    PAUSE
```

**No motor/stepper/TMC section** — the firmware owns the motor; the host only reads `PB7`.

### Power

Provide **24 V before USB** (or have it present at boot): the TMC is configured on the first task iteration, so it must be powered then.

---

## TMC2208 register values

Derived directly from the parameters of Mellow's firmware (via the `TMCStepper` library it uses), not guessed:

| Register | Value | Notes |
|---|---|---|
| GCONF | `0x000001C4` | spreadCycle, pdn_disable, mstep_reg_select (shaft bit set for FORWARD) |
| CHOPCONF | `0x12020055` | toff=5, vsense, 64 µsteps, intpol |
| IHOLD_IRUN | `0x00010F07` | IRUN=15, IHOLD=7 (≈ `rms_current(500)` @ Rsense 0.11) |
| PWMCONF | `0xC10D0024` | pwm_autoscale |
| VACTUAL | `77575` | 260 rpm, 64 µsteps |

---

## Status / disclaimer

Validated on **one setup** (FLY LLL Buffer Plus, STM32F072CB): auto-feed, manual buttons and runout-pause all working. Pins and register values are specific to this board. Use at your own risk — it's firmware on your hardware.

## Credits

- **[Fly3DTeam / Mellow](https://github.com/Fly3DTeam/Buffer)** — original FLY Buffer firmware & hardware (state machine ported from their open source).
- **[Klipper](https://github.com/Klipper3d/klipper)** — MCU firmware framework.
- **[Katapult](https://github.com/Arksine/katapult)** — USB bootloader.
- Community Klipper integrations: [river29](https://github.com/river29/Mellow-LLLBufferPLUS-klipper), [ss1gohan13](https://github.com/ss1gohan13/BufferPLUS-klipper).

## License

[GPL-3.0](LICENSE) — derived from GPL-3.0 sources (Fly3DTeam/Buffer, Klipper).
