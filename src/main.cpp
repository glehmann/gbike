#include <Arduino.h>
#include <limits.h>

int RXLED = 17;  // The RX LED has a defined Arduino pin
// Note: The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)

#define ACCELERATOR_PIN A3
#define ROTOR_PIN1 9
#define OUTPUT_PIN1 8

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
#define ACCELERATOR_MIN 250
#define ACCELERATOR_MAX 802
int acceleratorValues[ACCELERATOR_SAMPLES];
int acceleratorIndex = 0;
unsigned int accelerator = 0;
unsigned int acceleratorMin = UINT_MAX;
unsigned int acceleratorMax = 0;

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

void setOutput1(bool status) {
  outputStatus1 = status;
  if (status) {
    digitalWrite(OUTPUT_PIN1, LOW);
    digitalWrite(RXLED, LOW);   // set the RX LED ON
  } else {
    digitalWrite(OUTPUT_PIN1, HIGH);
    digitalWrite(RXLED, HIGH);    // set the RX LED OFF
  }
}

void onRotorRising1() {
  if (accelerator < 10) {
    setOutput1(true);
  }
}

void onRotorFalling1() {
  if (accelerator < 10) {
    setOutput1(false);
  }
}

unsigned int interval() {
  return 10 * 1024 / accelerator;
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

  pinMode(ROTOR_PIN1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTOR_PIN1), onRotorRising1, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTOR_PIN1), onRotorFalling1, FALLING);

  lastOutputSwitchT = millis();
}

void loop() {
  unsigned long t = millis();
  updateLoopStats(t);
  readAccelerator();
  if (accelerator >= 10) {
    if (t - lastOutputSwitchT >= 1000) {
      lastOutputSwitchT = t;
      setOutput1(true);
    }
    if (t - lastOutputSwitchT >= interval()) {
      lastOutputSwitchT = t;
      setOutput1(!outputStatus1);
    }
  }
  printStats(t);
  // delayMicroseconds(500);
}
