#include <Arduino.h>
#include "Motors.hpp"
#include "Sensors.hpp"
#include "Wifi.hpp"
#include "Logger.hpp"

// ======= PIN CONFIG =======
#define LED_PIN   2

// ======= TUNING =======
int   baseSpeed     = 120;     // fart i svinger
int   maxSpeed      = 140;     // fart paa rettstrekning
int   diamondSpeed  = 90;      // fart gjennom diamant
int   regSpeed      = 255;
float kp            = 0.0477f;
float ki            = 0.0f;
float kd            = 0.0010f;
bool  running       = false;
bool  diamondEnabled = true;   // toggleable - kan slas av paa WiFi

// ======= TILSTANDSMASKIN =======
enum State { STATE_CURVE, STATE_STRAIGHT, STATE_DIAMOND };
State currentState = STATE_CURVE;

// Terskler for STRAIGHT/CURVE
const float STRAIGHT_ERROR_THRESHOLD = 400.0f;
const float CURVE_ERROR_THRESHOLD    = 800.0f;
const int   STRAIGHT_SAMPLES_NEEDED  = 10;
int straightCounter = 0;

// Terskler for DIAMANT
const uint16_t SENSOR_ON_THRESHOLD     = 800;
const int      DIAMOND_SAMPLES_NEEDED  = 2;       // krev 2 samples for trygg deteksjon
const unsigned long DIAMOND_DURATION_MS = 600;
const unsigned long DIAMOND_COOLDOWN_MS = 1500;
int diamondCounter = 0;
unsigned long diamondStartTime = 0;
unsigned long lastDiamondEnd   = 0;

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
    wifi.setDiamondFlag(&diamondEnabled);

    digitalWrite(LED_PIN, HIGH);
    Serial.println("Klar! Koble til WiFi: LinjefolgerG11 / 12345678");
}

void loop() {
    wifi.handle();

    static bool wasRunning = false;

    if (!running) {
        motors.stop();
        wasRunning = false;
        currentState = STATE_CURVE;
        straightCounter = 0;
        diamondCounter = 0;
        return;
    }

    if (!wasRunning) {
        integral = 0.0f;
        lastError = 0.0f;
        lastTime = millis();
        wasRunning = true;
        currentState = STATE_CURVE;
        straightCounter = 0;
        diamondCounter = 0;
        lastDiamondEnd = 0;
    }

    // Les posisjon
    uint16_t pos = sensor.readPosition();

    // ===== SMART DIAMANT-DETEKSJON =====
    // Sjekk om BAADE ytter-venstre OG ytter-hoyre sensorer er aktive samtidig.
    // Det skjer kun ved kryss eller diamanthjorne, ikke i vanlige svinger.
    bool leftEdgeActive  = (sensor.values[0] > SENSOR_ON_THRESHOLD)
                        || (sensor.values[1] > SENSOR_ON_THRESHOLD);
    bool rightEdgeActive = (sensor.values[7] > SENSOR_ON_THRESHOLD)
                        || (sensor.values[8] > SENSOR_ON_THRESHOLD);
    bool diamondPattern  = leftEdgeActive && rightEdgeActive;

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

    // 1) DIAMANT-deteksjon - har prioritet
    bool canDetectDiamond = diamondEnabled
                            && currentState != STATE_DIAMOND
                            && (now - lastDiamondEnd > DIAMOND_COOLDOWN_MS);

    if (canDetectDiamond && diamondPattern) {
        diamondCounter++;
        if (diamondCounter >= DIAMOND_SAMPLES_NEEDED) {
            currentState = STATE_DIAMOND;
            diamondStartTime = now;
            straightCounter = 0;
            diamondCounter = 0;
        }
    } else if (currentState != STATE_DIAMOND) {
        diamondCounter = 0;
    }

    // 2) Vanlig CURVE/STRAIGHT-logikk hvis ikke i DIAMOND
    if (currentState == STATE_DIAMOND) {
        if (now - diamondStartTime >= DIAMOND_DURATION_MS) {
            currentState = STATE_CURVE;
            lastDiamondEnd = now;
            straightCounter = 0;
        }
    } else {
        switch (currentState) {
            case STATE_CURVE:
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
                if (absError > CURVE_ERROR_THRESHOLD) {
                    currentState = STATE_CURVE;
                    straightCounter = 0;
                }
                break;

            case STATE_DIAMOND:
                break;
        }
    }

    // Velg fart basert paa tilstand
    int targetSpeed;
    switch (currentState) {
        case STATE_STRAIGHT: targetSpeed = maxSpeed;     break;
        case STATE_DIAMOND:  targetSpeed = diamondSpeed; break;
        default:             targetSpeed = baseSpeed;    break;
    }

    int leftSpeed  = targetSpeed + corr;
    int rightSpeed = targetSpeed - corr;

    leftSpeed  = constrain(leftSpeed,  -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motors.setLeft(leftSpeed);
    motors.setRight(rightSpeed);

    logger.record((int16_t)error, (int16_t)corr);

    // Debug
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 100) {
        lastPrint = now;
        const char* stateStr = (currentState == STATE_STRAIGHT) ? "STR" :
                               (currentState == STATE_DIAMOND)  ? "DIA" : "CRV";
        Serial.print("["); Serial.print(stateStr); Serial.print("] ");
        Serial.print("Pos: "); Serial.print(pos);
        Serial.print(" | Err: "); Serial.print(error);
        Serial.print(" | Corr: "); Serial.print(corr);
        Serial.print(" | Spd: "); Serial.print(targetSpeed);
        Serial.print(" | LE:"); Serial.print(leftEdgeActive ? "1" : "0");
        Serial.print(" RE:"); Serial.println(rightEdgeActive ? "1" : "0");
    }
}