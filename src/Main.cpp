#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"

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
    wifi.begin();

    digitalWrite(LED_PIN, HIGH);
    Serial.println("Klar! Koble til WiFi: LinjefølgerG11 / 12345678");
}

void loop() {
    wifi.handle();

    static bool wasRunning = false;

    if (!running) {
        motors.stop();
        wasRunning = false;
        return;
    }

    // Rising edge: just started
    if (!wasRunning) {
        integral = 0.0f;
        lastError = 0.0f;
        lastTime = millis();
        wasRunning = true;
    }

    // Les posisjon
    uint16_t pos = sensor.readPosition();

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

    int leftSpeed  = baseSpeed + corr;
    int rightSpeed = baseSpeed - corr;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motors.setLeft(leftSpeed);
    motors.setRight(rightSpeed);

    // ===== LOGGER =====
    logger.record((int16_t)error, (int16_t)corr);

    // Debug — kun kvar 100 ms for å ikkje bremse loopen
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 100) {
        lastPrint = now;
        Serial.print("Pos: "); Serial.print(pos);
        Serial.print(" | Err: "); Serial.print(error);
        Serial.print(" | Corr: "); Serial.println(corr);
    }
}