#include <Arduino.h>
#include <limits.h>

int RXLED = 17;  // The RX LED has a defined Arduino pin
// Note: The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

#define ROTOR_PIN PIN3
#define ACCELERATOR_PIN A3

// time in milliseconds
unsigned long lastOutputSwitch = 0;
unsigned long lastStatOutput = 0;
bool status = false;
unsigned long lastLoop = 0;
unsigned long loopDurationMin = ULONG_MAX;
unsigned long loopDurationMax = 0;
unsigned long loopDurationSum = 0;
unsigned long loopCount = 0;

// accelerator reading
#define ACCELERATOR_SAMPLES 10
#define ACCELERATOR_MIN 250
#define ACCELERATOR_MAX 802
int acceleratorValues[ACCELERATOR_SAMPLES];
int acceleratorIndex = 0;
unsigned int accelerator = 0;
unsigned int acceleratorMin = UINT_MAX;
unsigned int acceleratorMax = 0;

// rotor reading
#define ROTOR_SAMPLES 4
volatile unsigned long rotorLastSwitch = 0;
volatile unsigned int rotorIntervals[ROTOR_SAMPLES];
volatile unsigned int rotorIntervalsIndex = 0;
unsigned int rotorInterval = UINT_MAX;
unsigned int rotorIntervalMin = UINT_MAX;

void readAccelerator() {
  unsigned int value = analogRead(ACCELERATOR_PIN);
  acceleratorMin = min(acceleratorMin, value);
  acceleratorMax = max(acceleratorMax, value);
  acceleratorValues[acceleratorIndex] = value;
  acceleratorIndex = (acceleratorIndex + 1) % ACCELERATOR_SAMPLES;
  float sum = 0;
  for(int i = 0; i < ACCELERATOR_SAMPLES; i++) {
    sum += acceleratorValues[i];
  }
  sum /= ACCELERATOR_SAMPLES;
  sum -= ACCELERATOR_MIN;
  sum *= 1024.0 / (ACCELERATOR_MAX - ACCELERATOR_MIN);
  sum = max(0.0, sum);
  sum = min(1024.0, sum);
  accelerator = (unsigned int)(sum);
}

void onRotorInterrupt() {
  unsigned long currentMillis = millis();
  if (rotorLastSwitch != 0) {
    rotorIntervals[rotorIntervalsIndex] = currentMillis - rotorLastSwitch;
    rotorIntervalMin = min(rotorIntervalMin, rotorIntervals[rotorIntervalsIndex]);
    rotorIntervalsIndex++;
  }
  rotorLastSwitch = currentMillis;
}

void updateRotorValues() {
  if (millis() - rotorLastSwitch > 500) {
    rotorInterval = UINT_MAX;
    rotorLastSwitch = 0;
    for(int i = 0; i < ROTOR_SAMPLES; i++) {
      rotorIntervals[i] = UINT_MAX;
    }
  }
  unsigned long sum = 0;
  for(int i = 0; i < ROTOR_SAMPLES; i++) {
    sum += rotorIntervals[i];
  }
  rotorInterval = sum / ROTOR_SAMPLES;
}

void switchOutput() {
  status = !status;
  if (status) {
    digitalWrite(RXLED, LOW);   // set the RX LED ON
    TXLED0; //TX LED is not tied to a normally controlled pin so a macro is needed, turn LED OFF
  } else {
    digitalWrite(RXLED, HIGH);    // set the RX LED OFF
    TXLED1; //TX LED macro to turn LED ON
  }
}

void resetOutput() {
  status = false;
  digitalWrite(RXLED, HIGH);
  TXLED0;
}

unsigned int interval() {
  return min(10 * 1024 / accelerator, rotorInterval);
}

void setup() {
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  // TX LED is set as an output behind the scenes

  Serial.begin(9600); //This pipes to the serial monitor
  Serial.println("Initialize Serial Monitor");

  for(int i = 0; i < ACCELERATOR_SAMPLES; i++) {
    acceleratorValues[i] = 0;
  }
  pinMode(ACCELERATOR_PIN, INPUT);

  for(int i = 0; i < ROTOR_SAMPLES; i++) {
    rotorIntervals[i] = UINT_MAX;
  }
  pinMode(ROTOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTOR_PIN), onRotorInterrupt, CHANGE);

  lastOutputSwitch = millis();
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long loopDuration = currentMillis - lastLoop;
  loopDurationSum += loopDuration;
  loopCount++;
  if (lastLoop != 0) {
    loopDurationMin = min(loopDurationMin, loopDuration);
    loopDurationMax = max(loopDurationMax, loopDuration);
  }
  lastLoop = currentMillis;
  readAccelerator();
  updateRotorValues();
  if (accelerator < 10 && rotorInterval == UINT_MAX) {
    resetOutput();
  } else {
    if (currentMillis - lastOutputSwitch >= interval()) {
      lastOutputSwitch = currentMillis;
      switchOutput();
    }
  }
  if (currentMillis - lastStatOutput >= 1000) {
    lastStatOutput = currentMillis;
    // accelerator
    Serial.print("accelerator: ");
    Serial.print(accelerator);
    Serial.print(", acceleratorMin: ");
    Serial.print(acceleratorMin);
    Serial.print(", acceleratorMax: ");
    Serial.println(acceleratorMax);
    // rotor
    Serial.print("rotorInterval: ");
    Serial.print(rotorInterval);
    Serial.print(", rotorIntervalMin: ");
    Serial.println(rotorIntervalMin);
    // loop
    Serial.print("loopDurationAvg: ");
    Serial.print(float(loopDurationSum) / loopCount);
    Serial.print(", loopDurationMin: ");
    Serial.print(loopDurationMin);
    Serial.print(", loopDurationMax: ");
    Serial.println(loopDurationMax);

    Serial.println();
  }
}
