/*
  Vivitar Vintage Radio volume knob and buttons to keyboard converter
  Huan Truong, 2019 <htruong@tnhh.net>
  Based on Rotary encoder example from 
  http://www.learningaboutelectronics.com/Articles/Rotary-encoder-circuit.php
  Runs on Arduino Pro Micro

  You will need to install the HID-Project library. 
  Click on Sketch - Include Library - Manage Libraries. 
  In the Filter box, type HID-Project. Click the Install button.
*/

#include <HID-Project.h>

const int encoderPinA = 14;
const int encoderPinB = 15;
const int pushBtnPin = A0;
const int delayMultimediaKeyMs = 500;
const int debouncerMs = 5;

unsigned long debounce = 0;

boolean encoderALast = LOW;
boolean encoderBLast = LOW;

void setup() {
  //Serial.begin(9600);
  //Serial.write("Welcome to Vivitar2kbd\n");
  // Serial.end();
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(pushBtnPin, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(pushBtnPin, HIGH);
  
  Consumer.begin();
}

void pressKey(uint16_t k, int d) {
  Consumer.write(k);
  if (d) {
    delay(d);
  }
}

void loop() {
  if ( millis() < debounce + debouncerMs) {
    return;
  }
  
  // Volume rotary
  boolean encoderA = digitalRead(encoderPinA);
  boolean encoderB = digitalRead(encoderPinB);

  if ((encoderA == HIGH) && (encoderB == HIGH)) {
    if ((encoderALast == LOW) && (encoderBLast == HIGH)) {
      pressKey(MEDIA_VOL_UP, 0);      
    } else if ((encoderBLast == LOW) && (encoderALast == HIGH)) {
      pressKey(MEDIA_VOL_DOWN, 0);
    }
  }
  
  encoderALast = encoderA;
  encoderBLast = encoderB;

  /****************************************/

  // Multimedia buttons
  int pushBtnRead = analogRead(pushBtnPin);
  if (pushBtnRead > 900 && debounce != 0) {
    debounce = 0;
  } else if (pushBtnRead <= 264) {
    if (pushBtnRead > 200) {
      pressKey(MEDIA_NEXT, delayMultimediaKeyMs); // Next
    } else if (pushBtnRead > 150) {
      pressKey(MEDIA_PREVIOUS, delayMultimediaKeyMs); // Prev
    } else if (pushBtnRead > 115) {
      pressKey(MEDIA_STOP, delayMultimediaKeyMs); // Stop/M
    } else {
      pressKey(MEDIA_PLAY_PAUSE, delayMultimediaKeyMs); // Play pause
    }
  }
  
  debounce = millis();
}
