#include <ESP32Encoder.h>
#include <Arduino.h>

#define encoderPinA 39
#define encoderPinB 34

ESP32Encoder encoder;

void encoderInit(){

  encoder.attachHalfQuad(encoderPinA, encoderPinB);

}

int detectMovement(){
  int64_t count = encoder.getCount();
  if (count >= 2){ // CW turn
    encoder.clearCount();
    return 1;
  }
  if (count <= -2){ // CCW Turn
    encoder.clearCount();
    return 2;
  }
  return 0;
}




