# 3D Printer Hotend Heater Controller

Arduino-based standalone heater controller for a 3D-printer hotend (thermistor + heater cartridge).

## Wiring

### Arduino Pin Connections

| Pin | Connected To           | Notes                          |
|-----|------------------------|--------------------------------|
| A1  | Thermistor voltage divider midpoint | See divider below       |
| D13 | MOSFET gate (heater)   | Also drives onboard LED         |
| D12 | MOSFET gate (fan)      | Fan on during cooling state     |
| D7  | Pushbutton → GND       | Internal pull-up, no resistor   |

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
                       Gate ── 10kΩ ── D13
                          │
                       Source ── GND
```

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
| 24V power supply | Heater |
| 12V power supply (or 24V tolerant fan) | Fan |

## Operation

Press the button to cycle states:

1. **IDLE** — everything off.
2. **HEATING** — heater PWM-controlled to reach 265°C with hysteresis ramp.
3. **COOLING** — heater off, fan on to cool down.
4. → back to IDLE.

## Safety

- Over-temperature cutoff at 300°C.
- Thermistor failure detection (ADC clamped to safe range).
- Heater power ramps down gradually as temperature approaches target to reduce overshoot.
