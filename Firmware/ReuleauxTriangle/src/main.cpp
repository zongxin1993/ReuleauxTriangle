
#include "Triangle.h"

Triangle triangle;

void setup() {
    Serial.begin(115200);
    timer = micros();
}

void loop() {
    triangle.mianloop();
}