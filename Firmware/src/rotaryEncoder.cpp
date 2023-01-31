#include <ESP32Encoder.h>
#include <Arduino.h>

#define encoderPinA 39
#define encoderPinB 34

ESP32Encoder encoder;

void encoderInit(){

  encoder.attachHalfQuad(encoderPinA, encoderPinB);

}

int64_t detectMovement(){
  int64_t count = encoder.getCount();
  if (count >= 2){ // CC turn
    return 2;
  }
  if (count <= -2){ // CCW Turn
    return 1;
  }
  return 0;
}




