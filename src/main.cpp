#include <Arduino.h>
#include <limits.h>

/* Pro Micro Test Code
   by: Nathan Seidle
   modified by: Jim Lindblom
   SparkFun Electronics
   date: September 16, 2013
   license: Public Domain - please use this code however you'd like.
   It's provided as a learning tool.

   This code is provided to show how to control the SparkFun
   ProMicro's TX and RX LEDs within a sketch. It also serves
   to explain the difference between Serial.print() and
   Serial1.print().
*/

int RXLED = 17;  // The RX LED has a defined Arduino pin
// Note: The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)


// time in milliseconds
unsigned long lastOutputSwitch = 0;
bool status = false;

// accelerator reading
int acceleratorValues[10];
int acceleratorIndex = 0;
unsigned int accelerator = 0;

// rotor reading
volatile unsigned long rotorLastSwitch = 0;
volatile unsigned int rotorInterval = UINT_MAX;

void readAccelerator() {
  acceleratorValues[acceleratorIndex] = analogRead(A3);
  acceleratorIndex = (acceleratorIndex + 1) % 10;
  float sum = 0;
  for(int i = 0; i < 10; i++) {
    sum += acceleratorValues[i];
  }
  sum /= 10;
  sum -= 250.0; // base value
  sum *= 1024.0 / (800.0 - 250.0); // scale to 0 - 1024
  sum = max(0.0, sum);
  sum = min(1024.0, sum);
  accelerator = (unsigned int)(sum);
}

void onRotorInterrupt() {
  unsigned long currentMillis = millis();
  if (rotorLastSwitch != 0) {
    rotorInterval = currentMillis - rotorLastSwitch;
  }
  rotorLastSwitch = currentMillis;
}

void detectRotorInactivity() {
  if (millis() - rotorLastSwitch > 500) {
    rotorInterval = UINT_MAX;
    rotorLastSwitch = 0;
  }
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

  for(int i = 0; i < 10; i++) {
    acceleratorValues[i] = 0;
  }

  pinMode(A3, INPUT);
  lastOutputSwitch = millis();

  attachInterrupt(digitalPinToInterrupt(PIN3), onRotorInterrupt, CHANGE);
}

void loop() {
  readAccelerator();
  detectRotorInactivity();
  if (accelerator < 10 && rotorInterval == UINT_MAX) {
    resetOutput();
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - lastOutputSwitch >= interval()) {
      lastOutputSwitch = currentMillis;
      switchOutput();
    }
  }
}
