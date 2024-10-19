/**
 * interrupts.h
 * Manejo de interrupciones para los pulsos digitales provenientes del acondicionamiento de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Si se presenta un overflow, manejarlo. Se puede guardar todo en memoria y liberar las variables o ...
 */
#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <Arduino.h>
#include "hardware_pins.h"
#include <stdint.h>  
#include <stdbool.h>

// #define DEBUG_INT

extern volatile uint16_t pulse_count1;  // Contador de pulsos
extern volatile uint16_t pulse_count2;
extern volatile bool detect1;  // Flag para indicar detección de pulso
extern volatile bool detect2;
extern volatile unsigned long pulse_start;
extern volatile unsigned long pulse_width;

void handlePulse1(void);
void handlePulse2(void);

void handlePulse1_FALLING(void);

#endif