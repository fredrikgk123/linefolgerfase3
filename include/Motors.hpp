#pragma once
#include <Arduino.h>

class Motors {
public:
    // Motor A (høyre)
    static const int AIN1 = 26;
    static const int AIN2 = 27;
    static const int PWMA = 25;

    // Motor B (venstre)
    static const int BIN1 = 18;
    static const int BIN2 = 5;
    static const int PWMB = 13;

    void begin();
    void setLeft(int speed);   // -255 til 255
    void setRight(int speed);  // -255 til 255
    void stop();
};