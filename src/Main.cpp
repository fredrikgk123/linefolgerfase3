#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed    = 100;
int   regSpeed     = 200;
float kp           = 0.0477f;
float ki           = 0.0f;
float kd           = 0.001f;
bool  running      = false;

// ======= STRAIGHT BOOST =======
const int   STRAIGHT_SPEED    = 130;
const float STRAIGHT_THRESHOLD = 200.0f;   // justér etter behov

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
}

void loop() {
    wifi.handle();

    static bool wasRunning = false;

    if (!running) {
        motors.stop();
        wasRunning = false;
        return;
    }

    if (!wasRunning) {
        integral  = 0.0f;
        lastError = 0.0f;
        lastTime  = millis();
        wasRunning = true;
    }

    uint16_t pos   = sensor.readPosition();
    float    error = (float)sensor.CENTER - (float)pos;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    integral += error * dt;
    integral  = constrain(integral, -1000.0f, 1000.0f);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float correction = kp * error + ki * integral + kd * derivative;
    int corr = (int)constrain(correction, -regSpeed, regSpeed);

    // Bruk høyere hastighet når roboten kjører rett frem
    int effective = (fabsf(error) < STRAIGHT_THRESHOLD) ? STRAIGHT_SPEED : baseSpeed;

    int leftSpeed  = effective + corr;
    int rightSpeed = effective - corr;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motors.setLeft(leftSpeed);
    motors.setRight(rightSpeed);

    logger.record((int16_t)error, (int16_t)corr);
}