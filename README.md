# 3D Printer Hotend Heater Controller

Arduino-based standalone heater controller for a 3D-printer hotend (thermistor + heater cartridge).

## Wiring

### Arduino Pin Connections

| Pin | Connected To           | Notes                          |
|-----|------------------------|--------------------------------|
| A1  | Thermistor voltage divider midpoint | See divider below (A0 unused — was unstable) |
| D11 | MOSFET gate (heater)   | **Hardware PWM** — proportional power for PID |
| D13 | Heating-status LED     | Onboard LED + optional external LED; ON while HEATING |
| D12 | MOSFET gate (fan)      | Fan on during cooling state     |
| D7  | Pushbutton → GND       | Internal pull-up, no resistor   |
| A4  | SSD1306 OLED — SDA     | I2C data |
| A5  | SSD1306 OLED — SCL     | I2C clock |

### Thermistor Voltage Divider

```
5V ──┬── 4.3kΩ resistor ──┬── Thermistor (100k NTC) ── GND
     │                    │
     └────────────────────┘
                         │
                       Pin A1
```

### Heater MOSFET Circuit

```
24V ── Heater Cartridge ──┬── Drain (IRLZ44N or similar) ── GND
                          │
                       Gate ── 10kΩ ── D11   (hardware PWM)
                          │
                       Source ── GND
```

> **Why D11?** D13 is *not* a hardware-PWM pin on the Uno — `analogWrite(13,…)`
> only gives full ON/OFF, so it can't deliver the partial power PID needs.
> Pins 3, 5, 6, 9, 10, 11 are the real PWM pins. D11 (Timer2) is used here and
> does not interfere with `millis()`. ~490 Hz switching is harmless for a
> resistive heater. (On a Leonardo, move heater to a PWM pin and note I2C is on
> D2/D3, not A4/A5.)

### Status LED (D13)

```
D13 ── (220Ω) ── external LED ── GND      (optional; onboard LED works too)
```

Lit whenever the controller is in the HEATING state.

### Fan MOSFET Circuit

```
12V ── Fan ──┬── Drain (IRLZ44N or similar) ── GND
             │
          Gate ── 10kΩ ── D12
             │
          Source ── GND
```

### Button

```
D7 ── pushbutton ── GND
```

No external pull-up needed — `INPUT_PULLUP` enabled in code.

### SSD1306 OLED (128×64, I2C)

```
  OLED          Uno
  GND ───────── GND
  VCC ───────── 5V
  SDA ───────── A4
  SCL ───────── A5
```

4 wires, no resistors (pull-ups are on the module). The common blue/white
0.96" module is 3.3–5V tolerant; use **5V** on the Uno for solid I2C levels.
Default I2C address `0x3C` (some are `0x3D`). Uses the **U8g2** library in
page-buffer mode (~128 B RAM). Shows live temp, setpoint, PWM, and state.

### Power

- Arduino powered via USB or 5V from PSU.
- Heater cartridge powered from **24V supply** (separate from Arduino 5V).
- Fan powered from **12V supply** (or 24V if fan rated for it).

## Bill of Materials

| Item | Notes |
|------|-------|
| Arduino Uno (or Leonardo) | |
| 100k NTC thermistor | Ender 3 S1 hotend |
| 4.3kΩ resistor | Measured — two resistors in series |
| Heater cartridge (24V) | |
| Fan (12V or 24V) | |
| 2× IRLZ44N N-channel MOSFETs | Logic-level gate |
| 2× 10kΩ resistors | Gate pulldown |
| Pushbutton (momentary) | SPST normally open |
| LED + 220Ω resistor | Optional external heating indicator on D13 |
| SSD1306 128×64 OLED (I2C) | Wires to A4/A5; needs U8g2 library |
| 24V power supply | Heater |
| 12V power supply (or 24V tolerant fan) | Fan |

## Operation

Press the button to cycle states:

1. **IDLE** — everything off.
2. **HEATING** — **PID-controlled** to hold the setpoint (default 275°C) flat.
3. **COOLING** — heater off, fan on to cool down.
4. → back to IDLE.

### Control: PID

The heater is held at the setpoint by a PID loop (computed every 250 ms) that
drives proportional power on D11. Unlike the old hysteresis band (which caused
a ±5°C sawtooth), PID settles on the exact power level needed and holds steady
(typically ±1–2°C once tuned).

- Derivative-on-measurement (no setpoint-change kick).
- Integral clamp (anti-windup).
- Default gains: `Kp=12.0  Ki=0.20  Kd=90.0` (starting point — tune for your hotend).

### Serial commands (9600 baud)

Type a command + Enter in the serial monitor:

| Command | Action                         | Example  |
|---------|--------------------------------|----------|
| `s<n>`  | Set target temperature (°C)    | `s290`   |
| `p<n>`  | Set Kp                         | `p15`    |
| `i<n>`  | Set Ki                         | `i0.3`   |
| `d<n>`  | Set Kd                         | `d80`    |
| `a`     | Run relay **auto-tune**        | `a`      |
| `?`     | Print current gains/setpoint   | `?`      |

**Setpoint is live-adjustable** — handy for dialing in PET, which crystallizes
and gets brittle too hot, but won't fuse too cold. Sweep `s260`…`s300` while
watching results, then settle on a number.

**Auto-tune** (`a`): the heater deliberately oscillates around the setpoint,
measures the response, and computes Kp/Ki/Kd automatically (Åström-Hägglund
relay method + Ziegler-Nichols). Runs ~6 cycles, prints the gains. It's a
blocking calibration — let it finish. Re-run after changing the setpoint a lot.

## Safety

- Hard over-temperature cutoff at **340°C** (forces IDLE).
- Thermistor failure detection (ADC clamped to safe range).
- PID output clamped 0–255; integral anti-windup prevents runaway.
- Auto-tune has its own overtemp + 10-minute timeout abort.
