#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"
#include "Encoder.hpp"
#include "PID.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed    = 120;
int   regSpeed     = 255;
float kp           = 0.0477f;
float ki           = 0.0f;
float kd           = 0.0010f;
bool  running      = false;

// ======= OBJEKTER =======
Motors    motors;
Sensor    sensor;
Logger    logger;
Encoder   encoder;
PIDController pid(kp, ki, kd);
RobotWifi wifi(running, baseSpeed, regSpeed, kp, ki, kd, sensor, logger);

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    motors.begin();
    sensor.begin();
    encoder.begin();
    wifi.begin();
    pid.begin();

    digitalWrite(LED_PIN, HIGH);
    Serial.println("Klar! Koble til WiFi: LinjefolgerG11 / 12345678");
}

void loop() {
    wifi.handle();

    // Oppdater RPM-beregning kontinuerlig
    encoder.update();

    if (!running) {
        motors.stop();
        return;
    }

    // Når roboten startes, nullstill PID
    static bool wasRunning = false;
    if (!wasRunning) {
        pid.reset();
        encoder.resetPulses();
        wasRunning = true;
    }
    if (!running) wasRunning = false;

    // Les posisjon
    uint16_t pos = sensor.readPosition();

    // Sjekk om linja er tapt - kun trigger ved 90-graders svinger
    // Krever at ALLE sensorene er under terskelen samtidig
    bool lineLost = true;
    for (uint8_t i = 0; i < Sensor::COUNT; i++) {
        if (sensor.values[i] > 800) { lineLost = false; break; }
    }

    if (lineLost) {
        // Nullstill PID når linja går tapt
        pid.reset();

        // Bruk siste kjente feil til å snu riktig vei
        float lastError = pid.getLastDerivative() > 0 ? 1.0f : -1.0f;
        if (lastError > 0) {
            // Linja var til venstre -> snu venstre
            motors.setLeft(-baseSpeed / 2);
            motors.setRight(baseSpeed / 2);
        } else {
            // Linja var til høyre -> snu høyre
            motors.setLeft(baseSpeed / 2);
            motors.setRight(-baseSpeed / 2);
        }
        return;
    }

    // PID - Beregn korrigering
    float error = (float)sensor.CENTER - (float)pos;
    float correction = pid.calculate(error);
    int corr = (int)constrain(correction, -regSpeed, regSpeed);

    int leftSpeed  = baseSpeed - corr;
    int rightSpeed = baseSpeed + corr;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motors.setLeft(leftSpeed);
    motors.setRight(rightSpeed);

    // ===== LOGGER: sensor =====
    logger.record((int16_t)error, (int16_t)corr);

    // ===== LOGGER: encoder =====
    logger.recordEncoder(
        encoder.getLeftPulses(),
        encoder.getRightPulses(),
        encoder.getLeftRPM(),
        encoder.getRightRPM()
    );

    // Debug - kun kvart 100 ms for aa ikkje bremse loopen
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    if (now - lastPrint >= 100) {
        lastPrint = now;
        Serial.printf("Pos:%u | Err:%.0f | Corr:%d | L_RPM:%.0f | R_RPM:%.0f\n",
                      pos, error, corr,
                      encoder.getLeftRPM(), encoder.getRightRPM());
    }
}
