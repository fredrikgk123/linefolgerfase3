#pragma once
#include <Arduino.h>
#include <algorithm>

/**
 * @class PIDController
 * @brief Klassisk PID-regulator (Proportional-Integral-Derivative)
 * 
 * Brukes til å regulere robotens bevegelse for å følge en linje.
 * Tar imot en feil (error) fra sensorposisjon og returnerer en korrigert motorpådrag.
 */
class PIDController {
public:
    /**
     * @brief Konstruktør for PID-regulatoren
     * @param kp Proporsjonal gevinst (standard: 0.05)
     * @param ki Integralgevinst (standard: 0.0)
     * @param kd Derivativ gevinst (standard: 0.3)
     */
    PIDController(float kp = 0.05f, float ki = 0.0f, float kd = 0.3f);

    /**
     * @brief Initialiserer PID-regulatoren
     * Nullstiller tilstanden (integral, derivative, lastError, lastTime)
     */
    void begin();

    /**
     * @brief Beregner PID-korrigering basert på feilen
     * @param error Posisjons-feilen fra linjefølgeren
     * @return Korrigert motorpådrag (-regSpeed til +regSpeed)
     */
    float calculate(float error);

    /**
     * @brief Setter PID-koeffisientene
     * @param kp Proporsjonal gevinst
     * @param ki Integralgevinst
     * @param kd Derivativ gevinst
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Henter nåværende PID-koeffisienter
     */
    void getGains(float& kp, float& ki, float& kd) const;

    /**
     * @brief Setter grenseverdier for integral-termen
     * @param minIntegral Minimum integral-verdi (standard: -1000)
     * @param maxIntegral Maksimum integral-verdi (standard: +1000)
     */
    void setIntegralLimits(float minIntegral, float maxIntegral);

    /**
     * @brief Nullstiller PID-tilstanden (integral, derivative, lastError)
     * Brukes når regulatoren startes på nytt
     */
    void reset();

    /**
     * @brief Henter gjeldende integral-verdi
     */
    float getIntegral() const { return integral; }

    /**
     * @brief Henter siste derivativ-verdi
     */
    float getLastDerivative() const { return lastDerivative; }

private:
    float kp, ki, kd;              // PID-koeffisienter
    float integral;                 // Akkumulert integralt
    float lastError;                // Forrige feil (for derivativ)
    float lastDerivative;           // Siste derivativ (for debugging)
    unsigned long lastTime;         // Forrige tidspunkt (ms)
    
    float minIntegral, maxIntegral; // Grenser for integral anti-windup
};

#endif
