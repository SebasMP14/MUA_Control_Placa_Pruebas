/**
 * interrupts.h
 * Manejo de interrupciones para los pulsos digitales provenientes del acondicionamiento de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
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

void handlePulse1_R(void);
void handlePulse2_R(void);
void activeInterrupt1(void);
void desactiveInterrupt1(void);
void activeInterrupt2(void);
void desactiveInterrupt2(void);

#endif