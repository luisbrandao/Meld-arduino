#include <math.h>
#include <U8g2lib.h>
#include <Wire.h>

// 128x64 SSD1306 over hardware I2C (Uno: SDA=A4, SCL=A5), page-buffer mode (~128B RAM).
// Default I2C address is 0x3C. If the screen stays blank, try adding in setup():
//   u8g2.setI2CAddress(0x3D * 2);   // some modules are at 0x3D
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ---------- Pins ----------
const int thermistorPin = A1;   // thermistor divider (A0 unused -- was behaving weird)
const int heaterPin     = 11;   // MOSFET gate -- HARDWARE PWM (moved from D13)
const int ledPin        = 13;   // heating-status LED (onboard + future external LED)
const int fanPin        = 12;   // cooling fan MOSFET
const int buttonPin     = 7;    // momentary button to GND (INPUT_PULLUP)

// ---------- Thermistor ----------
const float SERIES_RESISTOR     = 989.9;  // 1k pull-up (measured)
const float NOMINAL_RESISTANCE  = 100000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float BETA_COEFFICIENT    = 3950.0;

// ---------- Control parameters ----------
float TARGET_TEMP = 265.0;            // setpoint, adjustable live over serial ('s<n>')
const float OVERTEMP_CUTOFF = 320.0;  // hard safety cutoff
const int   PWM_MAX = 255;
const unsigned long PID_INTERVAL = 250; // ms between PID computations

// Safety / auto-cycle (manual process -- never leave it running unattended)
const unsigned long HEATING_MAX = 300000UL; // 5 min: auto-cycle HEATING -> COOLING
const unsigned long COOLING_MAX = 300000UL; // 5 min: auto-cycle COOLING -> IDLE
const float TEMP_MIN_VALID = 5.0;           // below this => thermistor open/disconnected

// ---------- PID gains ----------
// Derived from the trustworthy first auto-tune (Ku=60.59, Tu=17.02s) using
// Tyreus-Luyben tuning -> stable, low-overshoot hold (gentler than Ziegler-Nichols).
//   Kp = Ku/3.2,  Ki = Kp/(2.2*Tu),  Kd = Kp*(Tu/6.3)
// 'p'/'i'/'d' adjust live; re-run 'a' to retune (see note on AT_BAND below).
float Kp = 18.9;
float Ki = 0.51;
float Kd = 51.2;

// PID internal state
float integral = 0.0;
float lastTemp = 0.0;
bool  havePrevTemp = false;
unsigned long lastPidTime = 0;
int   lastPwm = 0;

// ---------- States ----------
enum State { IDLE, HEATING, COOLING };
State currentState = IDLE;
unsigned long stateStartTime = 0;
unsigned long lastSerialTime = 0;

// ---------- Button ----------
unsigned long buttonPressTime = 0;
bool lastButtonState = HIGH;
bool buttonPressed = false;

// ---------- Serial command buffer ----------
char serialBuf[16];
int  serialLen = 0;

float readTemperature() {
  int adc = analogRead(thermistorPin);
  if (adc >= 1023) adc = 1022;
  if (adc <= 0) adc = 1;

  float resistance = SERIES_RESISTOR * ((float)adc / (1023.0 - adc));

  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
  steinhart = 1.0 / steinhart;
  return steinhart - 273.15;
}

void setHeater(int value) {
  analogWrite(heaterPin, constrain(value, 0, PWM_MAX));
}

void setFan(bool on) {
  digitalWrite(fanPin, on ? HIGH : LOW);
}

void allOff() {
  setHeater(0);
  setFan(false);
}

void resetPID() {
  integral = 0.0;
  havePrevTemp = false;
  lastPidTime = millis();
  lastPwm = 0;
}

void printGains() {
  Serial.print("GAINS  setpoint=");
  Serial.print(TARGET_TEMP, 1);
  Serial.print("  Kp=");
  Serial.print(Kp, 3);
  Serial.print("  Ki=");
  Serial.print(Ki, 4);
  Serial.print("  Kd=");
  Serial.println(Kd, 3);
}

// ---------- PID ----------
// Derivative-on-measurement (no setpoint-change kick) + integral clamp (anti-windup).
int computePID(float temp, float dt) {
  float error = TARGET_TEMP - temp;
  float dTemp = havePrevTemp ? (temp - lastTemp) / dt : 0.0;

  integral += error * dt;
  // Anti-windup: clamp the integral so its contribution stays within output range.
  if (Ki > 0.0001) {
    float iLimit = PWM_MAX / Ki;
    integral = constrain(integral, -iLimit, iLimit);
  }

  float output = Kp * error + Ki * integral - Kd * dTemp;
  return (int)constrain(output, 0, PWM_MAX);
}

void updateHeating(float temp) {
  if (temp >= OVERTEMP_CUTOFF) {
    Serial.println("OVERTEMP CUTOFF!");
    allOff();
    currentState = IDLE;
    return;
  }

  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;
    if (dt <= 0.0) dt = PID_INTERVAL / 1000.0;

    int pwm = computePID(temp, dt);
    setHeater(pwm);

    lastTemp = temp;
    havePrevTemp = true;
    lastPidTime = now;
    lastPwm = pwm;
  }
}

void updateCooling() {
  setHeater(0);
  setFan(true);
}

// Runs every loop, before the state machine. Two jobs:
//   1) Open-thermistor guard: a disconnected sensor floats the divider and reads
//      implausibly cold (negative). If that happens, kill the heater -> IDLE.
//   2) Auto-cycle timeouts: HEATING -> COOLING -> IDLE after 5 min each, so the
//      rig never sits hot unattended.
void checkSafety(float temp) {
  if (temp < TEMP_MIN_VALID) {
    if (currentState != IDLE) {
      Serial.println("THERMISTOR FAULT (open?) -> IDLE");
    }
    allOff();
    currentState = IDLE;
    stateStartTime = millis();
    return;
  }

  unsigned long elapsed = millis() - stateStartTime;
  if (currentState == HEATING && elapsed > HEATING_MAX) {
    currentState = COOLING;
    stateStartTime = millis();
    Serial.println("TIMEOUT: HEATING -> COOLING");
  } else if (currentState == COOLING && elapsed > COOLING_MAX) {
    currentState = IDLE;
    stateStartTime = millis();
    Serial.println("TIMEOUT: COOLING -> IDLE");
  }
}

// ---------- OLED ----------
// Quick probe (one transaction) so a missing display costs ~1 timeout, not a
// full multi-page redraw worth of them. Keeps the control loop responsive.
bool displayPresent() {
  Wire.beginTransmission(0x3C);
  return (Wire.endTransmission() == 0);
}

void drawDisplay(float temp) {
  u8g2.firstPage();
  do {
    // Big current temperature
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor(0, 18);
    u8g2.print(temp, 1);
    u8g2.print(" C");

    // Setpoint + PWM
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.setCursor(0, 36);
    u8g2.print("Set ");
    u8g2.print(TARGET_TEMP, 0);
    u8g2.setCursor(72, 36);
    u8g2.print("PWM ");
    u8g2.print(currentState == HEATING ? (lastPwm * 100 + 127) / 255 : 0);
    u8g2.print("%");

    // State
    u8g2.setCursor(0, 54);
    u8g2.print("State: ");
    switch (currentState) {
      case IDLE:    u8g2.print("IDLE"); break;
      case HEATING: u8g2.print("HEATING"); break;
      case COOLING: u8g2.print("COOLING"); break;
    }
  } while (u8g2.nextPage());
}

// ---------- Relay auto-tune (Astrom-Hagglund + Ziegler-Nichols) ----------
// Blocking calibration: oscillates heater fully on/off around the setpoint,
// measures oscillation amplitude + period, derives Kp/Ki/Kd. Start with 'a'.
void runAutotune() {
  const int   AT_HIGH   = 255;       // relay high output
  const int   AT_LOW    = 0;         // relay low output
  const float AT_BAND   = 3.0;       // noise band (deg C); must exceed ADC step
                                     // (~1.3 deg C/count at operating temp) or the
                                     // relay chatters on quantization noise
  const int   AT_CYCLES = 6;         // oscillation cycles to observe
  const unsigned long AT_TIMEOUT = 600000UL; // 10 min safety

  Serial.println("AUTOTUNE: starting. Heater will oscillate around setpoint.");
  resetPID();

  bool relayHigh = true;
  setHeater(AT_HIGH);
  digitalWrite(ledPin, HIGH);

  float maxT = readTemperature();
  float minT = maxT;
  unsigned long lastPeakTime = 0;
  unsigned long periodSum = 0;
  int periodCount = 0;
  float ampSum = 0.0;
  int ampCount = 0;
  int switches = 0;
  unsigned long startTime = millis();

  while (switches < 2 * AT_CYCLES) {
    float temp = readTemperature();

    if (temp >= OVERTEMP_CUTOFF || (millis() - startTime) > AT_TIMEOUT) {
      setHeater(0);
      Serial.println("AUTOTUNE ABORTED (overtemp or timeout).");
      currentState = IDLE;
      return;
    }

    if (relayHigh) {
      if (temp > maxT) maxT = temp;
      if (temp > TARGET_TEMP + AT_BAND) {
        relayHigh = false;
        setHeater(AT_LOW);
        unsigned long now = millis();
        if (switches >= 2) { // skip initial transient
          if (lastPeakTime > 0) { periodSum += (now - lastPeakTime); periodCount++; }
          ampSum += (maxT - minT);
          ampCount++;
        }
        lastPeakTime = now;
        minT = temp;
        switches++;
      }
    } else {
      if (temp < minT) minT = temp;
      if (temp < TARGET_TEMP - AT_BAND) {
        relayHigh = true;
        setHeater(AT_HIGH);
        maxT = temp;
        switches++;
      }
    }

    // progress heartbeat
    unsigned long now = millis();
    if (now - lastSerialTime >= 1000) {
      lastSerialTime = now;
      Serial.print("AT temp=");
      Serial.print(temp, 1);
      Serial.print(" switches=");
      Serial.println(switches);

      if (displayPresent()) {
        u8g2.firstPage();
        do {
          u8g2.setFont(u8g2_font_6x12_tr);
          u8g2.setCursor(0, 20);
          u8g2.print("AUTOTUNE");
          u8g2.setCursor(0, 38);
          u8g2.print("T=");
          u8g2.print(temp, 1);
          u8g2.setCursor(0, 54);
          u8g2.print("cycle ");
          u8g2.print(switches / 2);
        } while (u8g2.nextPage());
      }
    }
    delay(20);
  }

  setHeater(0);

  float a  = (ampCount > 0)    ? (ampSum / ampCount) / 2.0               : 0.0; // amplitude
  float Tu = (periodCount > 0) ? (periodSum / (float)periodCount) / 1000.0 : 0.0; // sec
  float d  = (AT_HIGH - AT_LOW) / 2.0;

  if (a < 0.1 || Tu < 0.1) {
    Serial.println("AUTOTUNE FAILED (no clean oscillation). Gains unchanged.");
    currentState = IDLE;
    return;
  }

  float Ku = (4.0 * d) / (3.14159265 * a);
  float Tn = 0.5   * Tu;   // integral time
  float Tv = 0.125 * Tu;   // derivative time

  Kp = 0.6 * Ku;
  Ki = Kp / Tn;
  Kd = Kp * Tv;

  Serial.print("AUTOTUNE done. Ku=");
  Serial.print(Ku, 3);
  Serial.print(" Tu=");
  Serial.print(Tu, 2);
  Serial.println("s");
  printGains();

  resetPID();
  currentState = IDLE;
  Serial.println("STATE: IDLE");
}

// ---------- Serial commands ----------
// p<val> set Kp | i<val> set Ki | d<val> set Kd | s<val> set setpoint
// a       run auto-tune        | ?       print gains
void parseCommand(char *s) {
  if (s[0] == 0) return;
  Serial.print("> got: ");
  Serial.println(s);
  char cmd = s[0];
  float val = atof(s + 1);
  switch (cmd) {
    case 'p': case 'P': Kp = val; break;
    case 'i': case 'I': Ki = val; resetPID(); break;
    case 'd': case 'D': Kd = val; break;
    case 's': case 'S': TARGET_TEMP = val; break;
    case 'a': case 'A': runAutotune(); return;
    case '?': break;
    default:
      Serial.println("Commands: p<n> i<n> d<n> s<n> a ?");
      return;
  }
  printGains();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf[serialLen] = 0;
      parseCommand(serialBuf);
      serialLen = 0;
    } else if (serialLen < 15) {
      serialBuf[serialLen++] = c;
    }
  }
}

void handleButton() {
  bool reading = digitalRead(buttonPin);

  if (lastButtonState == HIGH && reading == LOW) {
    buttonPressTime = millis();
  }

  if (reading == LOW && (millis() - buttonPressTime > 50)) {
    if (!buttonPressed) {
      buttonPressed = true;
      switch (currentState) {
        case IDLE:
          currentState = HEATING;
          stateStartTime = millis();
          resetPID();
          Serial.println("STATE: HEATING");
          break;
        case HEATING:
          currentState = COOLING;
          stateStartTime = millis();
          Serial.println("STATE: COOLING");
          break;
        case COOLING:
          currentState = IDLE;
          stateStartTime = millis();
          Serial.println("STATE: IDLE");
          break;
      }
    }
  }

  if (reading == HIGH) {
    buttonPressed = false;
  }

  lastButtonState = reading;
}

void setup() {
  Serial.begin(9600);

  pinMode(heaterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  allOff();
  digitalWrite(ledPin, LOW);
  lastButtonState = digitalRead(buttonPin);

  // SAFETY: give I2C a hard timeout so a disconnected/faulty OLED can NEVER hang
  // loop(). Without this, Wire blocks forever waiting for an ACK -- if that happens
  // mid-HEATING, the heater stays on and the over-temp cutoff never runs.
  Wire.begin();
  Wire.setWireTimeout(25000, true);   // 25 ms timeout, auto-reset the bus

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 20);
    u8g2.print("Heater PID v3");
    u8g2.setCursor(0, 40);
    u8g2.print("ready");
  } while (u8g2.nextPage());

  Serial.println("Heater controller v3 (PID) ready");
  printGains();
  Serial.println("STATE: IDLE");
}

void loop() {
  handleButton();
  handleSerial();

  float temp = readTemperature();

  checkSafety(temp);

  switch (currentState) {
    case IDLE:
      allOff();
      break;
    case HEATING:
      updateHeating(temp);
      break;
    case COOLING:
      updateCooling();
      break;
  }

  // Heating-status LED (onboard D13 + future external LED)
  digitalWrite(ledPin, currentState == HEATING ? HIGH : LOW);

  unsigned long now = millis();
  if (now - lastSerialTime >= 500) {
    lastSerialTime = now;

    if (displayPresent()) drawDisplay(temp);

    Serial.print("ADC: ");
    Serial.print(analogRead(thermistorPin));
    Serial.print(" | Temp: ");
    Serial.print(temp, 1);
    Serial.print(" C | Set: ");
    Serial.print(TARGET_TEMP, 0);
    Serial.print(" | PWM: ");
    Serial.print(currentState == HEATING ? lastPwm : 0);
    Serial.print(" | Fan: ");
    Serial.print(currentState == COOLING ? "ON" : "OFF");
    Serial.print(" | State: ");
    switch (currentState) {
      case IDLE:    Serial.print("IDLE"); break;
      case HEATING: Serial.print("HEATING"); break;
      case COOLING: Serial.print("COOLING"); break;
    }
    Serial.println();
  }

  delay(50);
}
