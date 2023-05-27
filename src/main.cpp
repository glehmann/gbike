#include <Arduino.h>
#include <limits.h>

int RXLED = 17;  // The RX LED has a defined Arduino pin
// Note: The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

#define ACCELERATOR_PIN A3
#define ROTOR_PIN1 3
#define OUTPUT_PIN1 8
#define OUTPUT_ON_RATIO 1.0/3

// output status
bool outputStatus1 = false;
bool outputStatus2 = false;

// time in milliseconds
unsigned long lastOutputSwitchT = 0;
unsigned long lastStatOutput = 0;
unsigned long lastLoopT = 0;
unsigned long loopDurationMin = ULONG_MAX;
unsigned long loopDurationMax = 0;
unsigned long loopDurationSum = 0;
unsigned long loopCount = 0;

// accelerator reading
#define ACCELERATOR_SAMPLES 10
#define ACCELERATOR_MIN 260
#define ACCELERATOR_MAX 802
int acceleratorValues[ACCELERATOR_SAMPLES];
int acceleratorIndex = 0;
unsigned int accelerator = 0;
unsigned int acceleratorMin = UINT_MAX;
unsigned int acceleratorMax = 0;

// rotor reading
#define ROTOR_SAMPLES 10
int rotorValues[ROTOR_SAMPLES];
int rotorIndex = 0;
unsigned int rotor = LOW;

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

void readRotor() {
  unsigned int value = digitalRead(ROTOR_PIN1);
  rotorValues[rotorIndex] = value;
  rotorIndex = (rotorIndex + 1) % ROTOR_SAMPLES;
  int count = 0;
  for(int i = 0; i < ROTOR_SAMPLES; i++) {
    count += rotorValues[i] == true ? 1 : 0;
  }
  if (count == ROTOR_SAMPLES) {
    rotor = HIGH;
  } else if (count == 0) {
    rotor = LOW;
  }
}

void setOutput1(bool status) {
  outputStatus1 = status;
  if (status) {
    digitalWrite(OUTPUT_PIN1, HIGH);
    digitalWrite(RXLED, LOW);   // set the RX LED ON
  } else {
    digitalWrite(OUTPUT_PIN1, LOW);
    digitalWrite(RXLED, HIGH);    // set the RX LED OFF
  }
}

unsigned int interval() {
  float v = (accelerator * (720.0-60) / 1024.0) + 60;
  return (20 * 1024 / v) * (outputStatus1 ? OUTPUT_ON_RATIO : (1 - OUTPUT_ON_RATIO));
}

void printStats(unsigned long t) {
  if (t - lastStatOutput >= 1000) {
    lastStatOutput = t;
    // accelerator
    Serial.print("accelerator: ");
    Serial.print(accelerator);
    Serial.print(", acceleratorMin: ");
    Serial.print(acceleratorMin);
    Serial.print(", acceleratorMax: ");
    Serial.println(acceleratorMax);
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

void updateLoopStats(unsigned long t) {
  unsigned long loopDuration = t - lastLoopT;
  loopDurationSum += loopDuration;
  loopCount++;
  if (lastLoopT != 0) {
    loopDurationMin = min(loopDurationMin, loopDuration);
    loopDurationMax = max(loopDurationMax, loopDuration);
  }
  lastLoopT = t;
}

void setup() {
  pinMode(OUTPUT_PIN1, OUTPUT);
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output

  Serial.begin(9600); //This pipes to the serial monitor

  for(int i = 0; i < ACCELERATOR_SAMPLES; i++) {
    acceleratorValues[i] = 0;
  }
  pinMode(ACCELERATOR_PIN, INPUT);

  for(int i = 0; i < ROTOR_SAMPLES; i++) {
    rotorValues[i] = 0;
  }
  pinMode(ROTOR_PIN1, INPUT_PULLUP);

  lastOutputSwitchT = millis();
}

void loop() {
  unsigned long t = millis();
  updateLoopStats(t);
  readAccelerator();
  readRotor();
  if (accelerator >= 10) {
    if (t - lastOutputSwitchT >= 1000) {
      lastOutputSwitchT = t;
      setOutput1(true);
    }
    if (t - lastOutputSwitchT >= interval()) {
      lastOutputSwitchT = t;
      setOutput1(!outputStatus1);
    }
  } else {
    setOutput1(rotor);
  }
  printStats(t);
  // delayMicroseconds(500);
}
