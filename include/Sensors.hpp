#pragma once
#include <Arduino.h>
#include <QTRSensors.h>

class Sensor {
public:
    static const uint8_t  COUNT  = 9;
    static const uint16_t CENTER = 4000;  // (9-1)*1000 / 2

    void begin();
    void calibrate(uint16_t duration_ms = 5000);
    uint16_t readPosition();
    void printValues();
    uint16_t values[COUNT];

private:
    QTRSensors qtr;
    bool calibrated = false;
};