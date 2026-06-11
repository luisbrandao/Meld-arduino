# Arduino Heater Controller — Pin & Connection Reference

## Pin Summary (Objective)

| **Pin** | **Type** | **Connected To** | **Function** | **Notes** |
|---------|----------|------------------|--------------|-----------|
| A1 | Analog Input | Thermistor divider | Temperature reading | 16× oversampled, Steinhart-Hart conversion |
| A4 (SDA) | I2C Data | SSD1306 OLED display | Display communication | Hardware I2C, 5V tolerant |
| A5 (SCL) | I2C Clock | SSD1306 OLED display | Display communication | Hardware I2C, 5V tolerant |
| 2 | Digital Input | EC11 encoder phase A | Encoder rotation (INT0) | Input pullup, falling-edge interrupt |
| 3 | Digital Input | EC11 encoder phase B | Encoder rotation direction | Input pullup, read in ISR |
| 7 | Digital Input | Pushbutton → GND | State machine control | Input pullup, debounced, toggles IDLE→HEATING→COOLING→IDLE |
| 11 | PWM Output | MOSFET gate (heater) | Heater power control | Hardware PWM (Timer 1), 255-level PID control |
| 12 | Digital Output | MOSFET gate (fan) | Cooling fan control | Simple ON/OFF, HIGH = fan on |
| 13 | Digital Output | Status LED | Heating indicator | Onboard LED; HIGH while HEATING |

---

## Connection Diagram

```
                    ┌─────────────────────────────────┐
                    │      ARDUINO UNO                 │
                    │                                  │
        5V ─────────┼─────────────────────────────────┤
        GND ────────┼─────────────────────────────────┤
                    │                                  │
   ┌────────────────┼──────────────────────────────────┤
   │                │     INPUTS (with pullups)        │
   │     ┌──────────┼─ D2  ┌─────────────────────────── EC11 Encoder ─┐
   │     │          │      │  Phase A (S1)             │
   │     │   EC11   │      │                           │ Pushbutton
   │     │ Encoder  ├─ D3  ├─────────────────────────── ┤ (KEY)
   │     │  +Push   │      │  Phase B (S2)             │
   │     │          │      │                           │
   │     │          ├─ D7  ├───────────────────────────┤ → GND
   │     │          │      │  Push switch → GND        │
   │     └──────────┼──────┴─────────────────────────── ┘
   │                │
   │          ┌─────┼──────────────────────────────────┐
   │          │     │   I2C (to OLED display)          │
   │     SDA  ├─────┼─ A4 ─ SDA (no pullup needed)     │
   │          │     │                                  │
   │     SCL  ├─────┼─ A5 ─ SCL (no pullup needed)     │
   │          │     │      GND                         │
   │     GND  ├─────┼─ GND                             │
   │          └─────┼──────────────────────────────────┘
   │                │
   │          ┌─────┼──────────────────────────────────┐
   │          │     │   POWER OUTPUTS (HIGH = ON)      │
   │     D11  ├─────┼─ D11 ─ Heater MOSFET gate       │
   │ (PWM)    │     │       (0-255 PWM)                │
   │          │     │                                  │
   │     D12  ├─────┼─ D12 ─ Fan MOSFET gate          │
   │          │     │       (ON/OFF)                   │
   │          │     │                                  │
   │     D13  ├─────┼─ D13 ─ Status LED                │
   │          │     │       (ON = HEATING)             │
   │          └─────┼──────────────────────────────────┘
   │                │
   │        A1 ─────┼─ A1 ─ Thermistor divider output
   │        (ADC)   │
   │                │
   └────────────────┼─────────────────────────────────┘
                    └─────────────────────────────────┘


HEATER MOSFET (24V):
  24V ── Heater Cartridge ── Drain (IRLZ44N)
                             │
                            Gate ── 10kΩ ── D11 (PWM)
                             │
                           Source ── GND


FAN MOSFET (12V):
  12V ── Fan ── Drain (IRLZ44N)
               │
              Gate ── 10kΩ ── D12
               │
             Source ── GND


THERMISTOR DIVIDER (A1):
  5V ── 4.3kΩ resistor ── 100k NTC Thermistor ── GND
                               │
                              A1 (to ADC)


OLED I2C (SSD1306):
  GND ──────────────── GND
  VCC ──────────────── 5V
  SDA ──────────────── A4 (no external pullup needed)
  SCL ──────────────── A5 (no external pullup needed)
```

---

## State Transitions & Control

| **State** | **Button Press** | **Encoder** | **Safety** |
|-----------|------------------|-------------|-----------|
| **IDLE** | → HEATING | Ignored | Everything OFF |
| **HEATING** | → COOLING | ±1°C per detent (clamped) | Auto → COOLING after 2.5 min; hard cutoff at 340°C |
| **COOLING** | → IDLE | Ignored | Auto → IDLE when T < 110°C |

---

## New Component: EC11 Rotary Encoder

The code now includes the EC11 encoder (not in the old README):
- **Pin 2 (A)** — interrupt on falling edge, detects rotation
- **Pin 3 (B)** — read direction (CW vs CCW)
- **Push switch (KEY)** — tied to GND via Pin 7
- **Function:** Adjust setpoint ±1°C per detent while HEATING only

---

## Soldering Checklist

- [ ] **Thermistor divider** wired to A1 (5V → 4.3kΩ → A1 → 100k thermistor → GND)
- [ ] **EC11 encoder A** (S1) → Pin 2 with pullup
- [ ] **EC11 encoder B** (S2) → Pin 3 with pullup
- [ ] **EC11 push switch** (KEY) → Pin 7 with pullup (normally open to GND)
- [ ] **Heater MOSFET gate** (10kΩ pulldown) → Pin 11 (PWM)
- [ ] **Fan MOSFET gate** (10kΩ pulldown) → Pin 12
- [ ] **Status LED** → Pin 13 (or external LED + 220Ω resistor)
- [ ] **OLED SDA** → A4
- [ ] **OLED SCL** → A5
- [ ] **OLED GND** → GND
- [ ] **OLED VCC** → 5V
