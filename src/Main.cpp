#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed    = 100;   // 0-255
int   regSpeed     = 200;   // Maks svingehastighet 0-255
float kp           = 0.05f;
float ki           = 0.0f;
float kd           = 0.3f;
bool  running      = false;

// ======= OBJEKTER =======
Motors  motors;
Sensor  sensor;
RobotWifi wifi(running, baseSpeed, regSpeed, kp, ki, kd, sensor);

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

    // LED lyser når klar
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Klar! Koble til WiFi: LinjefølgerG11 / 12345678");
}

void loop() {
    wifi.handle();

    if (!running) {
        motors.stop();
        return;
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

    // Debug
    Serial.print("Pos: "); Serial.print(pos);
    Serial.print(" | Err: "); Serial.print(error);
    Serial.print(" | Corr: "); Serial.println(corr);
}