#pragma once
#include <Arduino.h>
#include "Sensors.hpp"

class RobotWifi {
public:
    RobotWifi(bool& running, int& baseSpeed, int& regSpeed,
              float& kp, float& ki, float& kd, Sensor& sensor);

    void begin();
    void handle();

private:
    bool&   running;
    int&    baseSpeed;
    int&    regSpeed;
    float&  kp;
    float&  ki;
    float&  kd;
    Sensor& sensor;

    void setupRoutes();
    String buildPage();
};