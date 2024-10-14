/**
 * interrupts.cpp
 * Manejo de interrupciones para los pulsos digitales provenientes del acondicionamiento de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Si se presenta un overflow, manejarlo. Se puede guardar todo en memoria y liberar las variables o ...
 */
#include "interrupts.h"

volatile uint16_t pulse_count1 = 0;  // Contador de pulsos
volatile uint16_t pulse_count2 = 0;
volatile bool detect1 = false;
volatile bool detect2 = false;
volatile unsigned long pulse_start_time = 0;
volatile unsigned long pulse_duration = 0;

void handlePulse1(void) {
    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_SWRST; // Resetear TC4
    while (TC4->COUNT16.SYNCBUSY.bit.SWRST);

    TC4->COUNT16.COUNT.reg = 0;
    while (TC4->COUNT16.SYNCBUSY.bit.COUNT);
    // pulse_start_time = 0;

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); // Esperar a que se habilite

    pulse_count1++;
    // detect1 = true;   // Indicar que se ha detectado un pulso

    // Cambiar a interrupción en flanco descendente
    attachInterrupt(digitalPinToInterrupt(PB08), handlePulse1_FALLING, FALLING);
}

void handlePulse2(void) {
    pulse_count2++;
    detect2 = true;
}

void handlePulse1_FALLING(void) {
    TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE; // Detener el TC
    while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); // Esperar a que se detenga
    
    uint16_t end_time = TC4->COUNT16.COUNT.reg;
    while (TC4->COUNT16.SYNCBUSY.bit.COUNT);

    #ifdef DEBUG_INT
    Serial.print("End time: ");
    Serial.println(end_time);
    #endif

    // Calcular duración
    pulse_duration = end_time * 0.01667; // Calcular la duración del pulso
    detect1 = true; // Indicar que se ha detectado un pulso

    // Cambiar de nuevo a interrupción en flanco ascendente
    attachInterrupt(digitalPinToInterrupt(PB08), handlePulse1, RISING);
}