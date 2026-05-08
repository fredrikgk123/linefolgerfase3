#pragma once
#include <Arduino.h>
#include "Sensors.hpp"
#include "Logger.hpp"

class RobotWifi {
public:
    RobotWifi(bool& running, int& baseSpeed, int& regSpeed,
              float& kp, float& ki, float& kd,
              Sensor& sensor, Logger& logger);

    void begin();
    void handle();

    // Pekar til diamond-flag i main, slik at vi kan toggle det
    void setDiamondFlag(bool* flag) { diamondEnabled = flag; }

private:
    String buildPage();
    void   setupRoutes();

    bool&   running;
    int&    baseSpeed;
    int&    regSpeed;
    float&  kp;
    float&  ki;
    float&  kd;
    Sensor& sensor;
    Logger& logger;

    bool  calibrating    = false;
    bool* diamondEnabled = nullptr;  // peker til main sin flag
};