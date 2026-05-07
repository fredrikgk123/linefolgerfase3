#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed    = 120;     // fart i svinger
int   maxSpeed     = 140;     // fart paa rettstrekning
int   regSpeed     = 255;
float kp           = 0.0477f;
float ki           = 0.0f;
float kd           = 0.0010f;
bool  running      = false;

// ======= TILSTANDSMASKIN =======
enum State { STATE_CURVE, STATE_STRAIGHT };
State currentState = STATE_CURVE;

// Terskler
const float STRAIGHT_ERROR_THRESHOLD = 400.0f;   // |err| under = rett
const float CURVE_ERROR_THRESHOLD    = 800.0f;   // |err| over  = sving
const int   STRAIGHT_SAMPLES_NEEDED  = 10;       // ~100 ms med liten feil
int straightCounter = 0;

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
        currentState = STATE_CURVE;
        straightCounter = 0;
        return;
    }

    // Rising edge: just started
    if (!wasRunning) {
        integral = 0.0f;
        lastError = 0.0f;
        lastTime = millis();
        wasRunning = true;
        currentState = STATE_CURVE;
        straightCounter = 0;
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

    // ===== TILSTANDSMASKIN =====
    float absError = fabs(error);

    switch (currentState) {
        case STATE_CURVE:
            // Sjekk om vi er paa rettstrekning
            if (absError < STRAIGHT_ERROR_THRESHOLD) {
                straightCounter++;
                if (straightCounter >= STRAIGHT_SAMPLES_NEEDED) {
                    currentState = STATE_STRAIGHT;
                }
            } else {
                straightCounter = 0;
            }
            break;

        case STATE_STRAIGHT:
            // Hvis feilen oker -> tilbake til CURVE umiddelbart
            if (absError > CURVE_ERROR_THRESHOLD) {
                currentState = STATE_CURVE;
                straightCounter = 0;
            }
            break;
    }

    // Velg fart basert paa tilstand
    int targetSpeed = (currentState == STATE_STRAIGHT) ? maxSpeed : baseSpeed;

    int leftSpeed  = targetSpeed + corr;
    int rightSpeed = targetSpeed - corr;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motors.setLeft(leftSpeed);
    motors.setRight(rightSpeed);

    // ===== LOGGER =====
    logger.record((int16_t)error, (int16_t)corr);

    // Debug - kvart 100 ms
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 100) {
        lastPrint = now;
        const char* stateStr = (currentState == STATE_STRAIGHT) ? "STR" : "CRV";
        Serial.print("["); Serial.print(stateStr); Serial.print("] ");
        Serial.print("Pos: "); Serial.print(pos);
        Serial.print(" | Err: "); Serial.print(error);
        Serial.print(" | Corr: "); Serial.print(corr);
        Serial.print(" | Spd: "); Serial.println(targetSpeed);
    }
}