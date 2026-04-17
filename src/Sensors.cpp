#include "Sensors.hpp"

void Sensor::begin() {
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){4, 17, 16, 19, 21, 22, 23, 32, 33}, COUNT);
}

void Sensor::calibrate(uint16_t duration_ms) {
    Serial.println("Kalibrering startar...");

    uint16_t cycles = duration_ms / 10;
    for (uint16_t i = 0; i < cycles; i++) {
        qtr.calibrate();
        yield();
        delay(10);

        if (i % (cycles / 5) == 0) {
            Serial.print("Kalibrerer... ");
            Serial.print((i * 100) / cycles);
            Serial.println("%");
        }
    }

    calibrated = true;
    Serial.println("Kalibrering ferdig!");
}

uint16_t Sensor::readPosition() {
    if (!calibrated) return CENTER;
    return qtr.readLineBlack(values);
}

void Sensor::printValues() {
    for (uint8_t i = 0; i < COUNT; i++) {
        Serial.print(values[i]);
        Serial.print('\t');
    }
}