#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"
#include "Encoder.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed    = 100;
int   regSpeed     = 200;
float kp           = 0.05f;
float ki           = 0.0f;
float kd           = 0.3f;
bool  running      = false;

// ======= OBJEKTER =======
Motors    motors;
Sensor    sensor;
Logger    logger;
Encoder   encoder;
RobotWifi wifi(running, baseSpeed, regSpeed, kp, ki, kd, sensor, logger);

// ======= PID STATE =======
float lastError  = 0.0f;
float integral   = 0.0f;
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    motors.begin();
    sensor.begin();
    encoder.begin();
    wifi.begin();

    digitalWrite(LED_PIN, HIGH);
    Serial.println("Klar! Koble til WiFi: LinjefolgerG11 / 12345678");
}

void loop() {
    wifi.handle();

    // Oppdater RPM-beregning kontinuerlig
    encoder.update();

    static bool wasRunning = false;

    if (!running) {
        motors.stop();
        wasRunning = false;
        return;
    }

    // Rising edge: just started
    if (!wasRunning) {
        integral  = 0.0f;
        lastError = 0.0f;
        lastTime  = millis();
        encoder.resetPulses();
        wasRunning = true;
    }

    // Les posisjon
    uint16_t pos = sensor.readPosition();

    // Sjekk om linja er tapt - kun trigger ved 90-graders svinger
    // Krever at ALLE sensorene er under terskelen samtidig
    bool lineLost = true;
    for (uint8_t i = 0; i < Sensor::COUNT; i++) {
        if (sensor.values[i] > 800) { lineLost = false; break; }
    }

    if (lineLost) {
        // Nullstill integral så den ikke hopper når linja gjenfinnes
        integral = 0.0f;
        lastTime = millis();

        // Bruk siste kjente feil til å snu riktig vei
        if (lastError > 0) {
            // Linja var til venstre -> snu venstre
            motors.setLeft(-baseSpeed / 2);
            motors.setRight(baseSpeed / 2);
        } else {
            // Linja var til hoyre -> snu hoyre
            motors.setLeft(baseSpeed / 2);
            motors.setRight(-baseSpeed / 2);
        }
        return;
    }

    // PID
    float error = (float)sensor.CENTER - (float)pos;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    integral += error * dt;
    integral = constrain(integral, -1000.0f, 1000.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = kp * error + ki * integral + kd * derivative;
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
    if (now - lastPrint >= 100) {
        lastPrint = now;
        Serial.printf("Pos:%u | Err:%.0f | Corr:%d | L_RPM:%.0f | R_RPM:%.0f\n",
                      pos, error, corr,
                      encoder.getLeftRPM(), encoder.getRightRPM());
    }
}