#include <Arduino.h>
#include <RF24.h>
#include <Bluepad32.h>


GamepadPtr myGamepads[2];

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  myGamepads[0]->brake();
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}