#include "PID.hpp"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      integral(0.0f), lastError(0.0f), lastDerivative(0.0f),
      lastTime(0), minIntegral(-1000.0f), maxIntegral(1000.0f) {}

void PIDController::begin() {
    reset();
    lastTime = millis();
}

float PIDController::calculate(float error) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    
    // Sikre at dt ikke er 0 eller negativ
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    // P-term: proporsjonal del
    float P = kp * error;

    // I-term: integral del med anti-windup
    integral += error * dt;
    integral = std::max(minIntegral, std::min(maxIntegral, integral));
    float I = ki * integral;

    // D-term: derivativ del
    lastDerivative = (error - lastError) / dt;
    float D = kd * lastDerivative;

    lastError = error;

    // Total PID-korrigering
    float correction = P + I + D;

    return correction;
}

void PIDController::setGains(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void PIDController::getGains(float& outKp, float& outKi, float& outKd) const {
    outKp = kp;
    outKi = ki;
    outKd = kd;
}

void PIDController::setIntegralLimits(float minI, float maxI) {
    minIntegral = minI;
    maxIntegral = maxI;
}

void PIDController::reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastDerivative = 0.0f;
    lastTime = millis();
}
