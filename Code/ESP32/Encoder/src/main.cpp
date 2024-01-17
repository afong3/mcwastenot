#include <Arduino.h>

volatile long encoderPosition = 0;
volatile bool indexSignalReceived = false;

const int pinA = 3;  // GPIO 3 for encoder A output
const int pinB = 34; // GPIO 34 for encoder B output
const int pinX = 33; // GPIO 33 for encoder X output (index)

void IRAM_ATTR onEncoderChange() {
  static byte lastStateA = LOW;
  byte currentStateA = digitalRead(pinA);
  byte currentStateB = digitalRead(pinB);
  
  if (currentStateA != lastStateA) { // If there's a change in state of pin A
    if (currentStateA == currentStateB) {
      encoderPosition++;
    } else {
      encoderPosition--;
    }
  }
  lastStateA = currentStateA;
}

void IRAM_ATTR onIndex() {
  indexSignalReceived = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinX, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pinA), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinX), onIndex, RISING);
}

void loop() {
  if (indexSignalReceived) {
    encoderPosition = 0;
    indexSignalReceived = false;
  }

  unsigned long time = millis();
  Serial.print(time);
  Serial.print(",");
  Serial.println(encoderPosition);
  
  delay(10);
}
