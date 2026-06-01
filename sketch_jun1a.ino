#include <math.h>

// Pins
const int thermistorPin = A1;
const int heaterPin = 13;
const int fanPin = 12;
const int buttonPin = 7;

// Thermistor
const float SERIES_RESISTOR = 4300.0;
const float NOMINAL_RESISTANCE = 100000.0;
const float NOMINAL_TEMPERATURE = 25.0;
const float BETA_COEFFICIENT = 3950.0;

// Control parameters
const float TARGET_TEMP = 265.0;
const float HYSTERESIS = 5.0;
const float OVERTEMP_CUTOFF = 300.0;
const int PWM_MAX = 255;
const unsigned long CYCLE_TIME = 2000;

// States
enum State { IDLE, HEATING, COOLING };
State currentState = IDLE;

// Timing
unsigned long stateStartTime = 0;
unsigned long lastCycleTime = 0;
unsigned long lastSerialTime = 0;
unsigned long buttonPressTime = 0;
bool lastButtonState = HIGH;
bool buttonPressed = false;

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

void updateHeating(float temp) {
  if (temp >= OVERTEMP_CUTOFF) {
    Serial.println("OVERTEMP CUTOFF!");
    allOff();
    currentState = IDLE;
    return;
  }

  int pwm;
  if (temp >= TARGET_TEMP + HYSTERESIS) {
    pwm = 0;
  } else if (temp <= TARGET_TEMP - HYSTERESIS) {
    pwm = PWM_MAX;
  } else {
    // Linear ramp within the hysteresis band
    float fraction = (temp - (TARGET_TEMP - HYSTERESIS)) / (2.0 * HYSTERESIS);
    fraction = constrain(fraction, 0.0, 1.0);
    pwm = round((1.0 - fraction) * PWM_MAX);
  }

  setHeater(pwm);
}

void updateCooling() {
  setHeater(0);
  setFan(true);
}

void setup() {
  Serial.begin(9600);

  pinMode(heaterPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  allOff();
  lastButtonState = digitalRead(buttonPin);

  Serial.println("Heater controller v2 ready");
  Serial.println("STATE: IDLE");
}

void loop() {
  handleButton();

  float temp = readTemperature();

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

  unsigned long now = millis();
  if (now - lastSerialTime >= 500) {
    lastSerialTime = now;

    Serial.print("ADC: ");
    Serial.print(analogRead(thermistorPin));
    Serial.print(" | Temp: ");
    Serial.print(temp, 1);
    Serial.print(" C | Heater: ");
    Serial.print(currentState == HEATING ? "ON" : "OFF");
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
