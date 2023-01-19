#include "keypad.h"
#include <Arduino.h>

#define R1 32
#define R2 33
#define R3 25
#define R4 26
#define C1 27
#define C2 14
#define C3 12
#define C4 16

int keypadRead() {
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      digitalWrite(C1, LOW);
      digitalWrite(C2, HIGH);
      digitalWrite(C3, HIGH);
      digitalWrite(C4, HIGH);
    }

    if (i == 1) {
      digitalWrite(C1, HIGH);
      digitalWrite(C2, LOW);
      digitalWrite(C3, HIGH);
      digitalWrite(C4, HIGH);
    }

    if (i == 2) {
      digitalWrite(C1, HIGH);
      digitalWrite(C2, HIGH);
      digitalWrite(C3, LOW);
      digitalWrite(C4, HIGH);
    }

    if (i == 3) {
      digitalWrite(C1, HIGH);
      digitalWrite(C2, HIGH);
      digitalWrite(C3, HIGH);
      digitalWrite(C4, LOW);
    }

    if (digitalRead(R1) == LOW) {
      if (digitalRead(C1) == LOW) {
        return 1;
      }
      else if (digitalRead(C2) == LOW) {
        return 2;
      }
      else if (digitalRead(C3) == LOW) {
        return 3;
      }
      else if (digitalRead(C4) == LOW) {
        return 4;
      }

    }

    if (digitalRead(R2) == LOW) {
      if (digitalRead(C1) == LOW) {
        return 5;
      }
      else if (digitalRead(C2) == LOW) {
        return 6;
      }
      else if (digitalRead(C3) == LOW) {
        return 7;
      }
      else if (digitalRead(C4) == LOW) {
        return 8;
      }
    }

    if (digitalRead(R3) == LOW) {
      if (digitalRead(C1) == LOW) {
        return 9;
      }
      else if (digitalRead(C2) == LOW) {
        return 10;
      }
      else if (digitalRead(C3) == LOW) {
        return 11;
      }
      else if (digitalRead(C4) == LOW) {
        return 12;
      }
    }

    if (digitalRead(R4) == LOW) {
      if (digitalRead(C1) == LOW) {
        return 13;
      }
      else if (digitalRead(C2) == LOW) {
        return 14;
      }
      else if (digitalRead(C3) == LOW) {
        return 15;
      }
      else if (digitalRead(C4) == LOW) {
        return 16;
      }
    }
  }
  return 0;
}