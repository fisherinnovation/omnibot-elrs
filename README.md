# Omnibot ELRS

ExpressLRS CRSF radio control for the [Crunchlabs Omnibot](https://www.crunchlabs.com/products/omnibot), replacing the stock binary RF board with a RadioMaster XR1 Nano ELRS receiver and a Radiomaster transmitter.

The stock firmware maps buttons to fixed speed vectors — everything is on or off. This firmware reads analog joystick values over CRSF and feeds them directly into the existing omni-directional kinematic model, giving smooth proportional control over all three degrees of freedom simultaneously.

## What changes

| | Stock firmware | This firmware |
|---|---|---|
| Receiver | Crunchlabs RF board (8-pin digital) | RadioMaster XR1 Nano ELRS (1 wire) |
| Control | Binary — buttons only | Analog — full joystick range |
| Diagonal movement | Special "strafe mode" required | Natural — push stick diagonally |
| Forklift | Full speed or stop | Proportional — stick position sets speed |
| Signal loss | No failsafe | Motors stop within 300 ms |
| Arm-on-boot | Moves immediately | Requires sticks at neutral for 1 s |

The kinematic model and motor control code from the original Crunchlabs firmware are preserved unchanged.

## Hardware

- Crunchlabs Omnibot (fully assembled)
- [RadioMaster XR1 Nano Multi-Frequency ELRS Receiver](https://www.radiomasterrc.com/products/xr1-nano-multi-frequency-expresslrs-receiver)
- Any Radiomaster transmitter with an ELRS module (TX16S, Boxer, Zorro, etc.)
- Three short wires (TX, 5V, GND)

## Wiring

Remove the stock RF board entirely. Connect the XR1 Nano to the Arduino:

```
XR1 Nano          Arduino Nano
─────────         ────────────
TX       ───────→ pin 0  (RX)
5V       ───────→ 5V
GND      ──────── GND
```

Pin 1 (Arduino TX) is not needed — leave it unconnected on the XR1 side.

**No signal inverter is needed.** CRSF is standard UART logic, not inverted like SBUS.

> **Important:** Disconnect the XR1 TX wire from Arduino pin 0 before uploading
> firmware. Hardware Serial is shared between USB upload and CRSF. Reconnect
> after the upload completes.

## One-time receiver setup

The XR1 Nano defaults to 420,000 baud. The Arduino Nano (ATmega328P) cannot reach that rate, so you need to change it once via the receiver's built-in WiFi interface:

1. Power the XR1 (connect it to the Arduino, then power the Arduino via USB)
2. On your phone or laptop, connect to the XR1's WiFi hotspot (check the XR1 docs for the network name)
3. Open the WebUI
4. Find the UART / baud rate setting
5. Change it to **115200**
6. Leave the protocol as **CRSF**
7. Save and let the receiver restart

You only need to do this once. The setting persists across power cycles.

## Building and uploading

1. Open `omnibot.ino` in the Arduino IDE (or VS Code with the Arduino extension)
2. Select board: **Arduino Nano** — processor: **ATmega328P**
3. **Disconnect the XR1 TX wire from Arduino pin 0**
4. Select the correct COM/serial port and upload
5. Reconnect the XR1 TX wire to pin 0

No additional libraries are required. The CRSF parser is self-contained.

## Control layout

Radiomaster Mode 2 (default):

```
Left stick                        Right stick
┌─────────────────┐               ┌─────────────────┐
│                 │               │        ↑        │
│   ←  Rotate  →  │               │   ← Strafe →    │
│                 │               │        ↓        │
│   ↑  Fork up    │               │   Forward/back  │
│   ↓  Fork down  │               │                 │
└─────────────────┘               └─────────────────┘
 Throttle axis                     Auto-centering
 (non-centering ratchet)
```

**Right stick** controls translation — the robot moves in whatever direction you push the stick while always keeping the same heading. Push diagonally for diagonal movement; the omni wheels handle it natively.

**Left stick X** rotates the robot in place. You can rotate and translate at the same time.

**Left stick Y** controls the forklift. Because this is the throttle axis (non-centering ratchet), the stick holds its position when you let go, so the fork runs at a fixed speed until you move the stick again. Center the stick to stop the fork.

## Arming sequence

On every power-on or signal recovery, the robot waits until **all four axes sit within the dead zone for one full second** before it will move. This prevents the robot from lurching if the transmitter was switched on with a stick displaced.

You will see/feel nothing happen for that first second — this is normal. Once armed, control is immediate.

## Tuning

All tuning parameters are in `config.h`. You should not need to edit `omnibot.ino`.

### If the robot moves the wrong direction

Set the relevant flag in `config.h`:

```cpp
bool flipM1 = false;  // flip if one drive wheel spins backwards
bool flipM2 = false;
bool flipM3 = false;
bool flipM4 = true;   // flip if fork goes down when you expect up
```

```cpp
const bool INV_DRIVE  = true;   // flip if forward goes backward
const bool INV_STRAFE = false;  // flip if left goes right
const bool INV_ROTATE = false;  // flip if CW goes CCW
const bool INV_FORK   = false;  // flip if up goes down
```

Test one axis at a time. Start with drive (forward/back) before diagonals.

### Speed

```cpp
uint8_t maxSpeed = 200;  // 0–255 PWM duty cycle
```

Start at 150–200 while tuning direction flags. Raise it once everything moves correctly.

### Dead zone

```cpp
const int CRSF_DEADBAND = 120;  // in the internal ±1000 scale (~12% of full throw)
```

**Raise** if the robot creeps slowly when sticks are released.  
**Lower** if the robot feels sluggish to respond near the stick centre.

### Channel assignments

If your transmitter uses a different mode or you want to remap controls:

```cpp
const uint8_t CH_STRAFE = 0;  // CRSF channel index (0-based) for strafe
const uint8_t CH_DRIVE  = 1;  // forward/back
const uint8_t CH_FORK   = 2;  // forklift
const uint8_t CH_ROTATE = 3;  // rotation
```

## Troubleshooting

**Robot won't arm (never moves)**
- Check that the XR1 is bound to your transmitter and the link LED is solid
- Confirm the XR1 baud rate is set to 115200 in the WebUI
- Verify the TX wire is connected to Arduino pin 0
- Center all sticks and wait a full second

**Robot moves immediately on power-on without waiting**
- This shouldn't happen with the arm-state logic, but if it does, check that `CRSF_ARM_MS = 1000` hasn't been set to 0

**Motors stop after ~300 ms and won't restart**
- The CRSF link dropped. Check wiring to pin 0, receiver power, and transmitter range
- The robot will re-arm automatically once a clean link is restored (sticks neutral for 1 s)

**One or more wheels spin the wrong way**
- Set the corresponding `flipM1`/`flipM2`/`flipM3` flag in `config.h`

**Forklift goes the wrong way**
- Set `flipM4 = false` (or `true` if it's currently `false`) in `config.h`

**Robot drifts slowly when sticks are centered**
- Increase `CRSF_DEADBAND` in `config.h` (try 150)

**Can't upload — serial port errors**
- Disconnect the XR1 TX wire from Arduino pin 0 before uploading

**Serial monitor shows garbage**
- The CRSF data stream is being printed to the monitor. Disconnect the XR1 TX wire from pin 0 to use the serial monitor

## How it works

The CRSF parser is a non-blocking byte-by-byte state machine. Each loop iteration drains whatever bytes have arrived in the hardware serial buffer and advances the parser state. A complete, CRC-validated frame updates the drive vector; between frames the last commanded speed is held. This keeps the motor PWM updates smooth with no blocking reads.

The kinematic model converts a three-element velocity vector `{vx, vy, ω}` into individual wheel speeds using the algebraic inverse kinematics for a three-wheeled omnidirectional robot. The relationship is linear, so a proportional joystick input directly produces a proportional wheel output — no special-casing for diagonals, and simultaneous translation and rotation work naturally.

For the full kinematic derivation, see:

> Siradjuddin, Indrazno. "Kinematics and control a three wheeled omnidirectional mobile robot." *Int. J. Electr. Electron. Eng* 6.12 (2019): 1–6.  
> https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf

## Credits

- [Crunchlabs](https://www.crunchlabs.com) — original Omnibot hardware and kinematic firmware foundation
- [RadioMaster](https://www.radiomasterrc.com) — XR1 Nano receiver and transmitter hardware
- [ExpressLRS](https://www.expresslrs.org) — open-source ELRS firmware
