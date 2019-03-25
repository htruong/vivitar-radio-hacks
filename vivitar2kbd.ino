/*
  Vivitar Vintage Radio volume knob and buttons to keyboard converter
  Written by Huan Truong, 2019 <htruong@tnhh.net>
  Based on Rotary encoder example from 
  http://www.learningaboutelectronics.com/Articles/Rotary-encoder-circuit.php
  Runs on Arduino Pro Micro
*/

#include <Keyboard.h>

const int encoderPinA = 14;
const int encoderPinB = 15;
const int pushBtnPin = A0;

unsigned long debounce = 0;

boolean encoderALast = LOW;

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(pushBtnPin, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(pushBtnPin, HIGH);
  Serial.begin(9600);
}

void pressKey(int k, bool prevent_repeat) {
  if (prevent_repeat && (millis() - debounce < 200)) {
    return;
  }
  Keyboard.write(k);
  debounce = millis();
}

void loop() {

  int pushBtnRead = analogRead(pushBtnPin);
  if (pushBtnRead > 900 && debounce != 0) {
    debounce = 0;
  } else if (pushBtnRead <= 264) {
    if (pushBtnRead > 200) {
      pressKey('b', true);
    } else if (pushBtnRead > 150) {
      pressKey('z', true);
    } else if (pushBtnRead > 115) {
      pressKey('v', true);
    } else {
      pressKey('x', true);
    }
  }

  boolean encoderA = digitalRead(encoderPinA);

  if ((encoderALast == HIGH) && (encoderA == LOW)) {
    if (digitalRead(encoderPinB) == LOW) {
      pressKey(KEY_RIGHT_ARROW, false);
    } else {
      pressKey(KEY_LEFT_ARROW, false);
    }
  }
  encoderALast = encoderA;
}
