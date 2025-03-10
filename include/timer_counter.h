/**
 * @file timer_counter.h
 *  Configuración y manejo de interrupción del timer counter (32bits). Se espera una interrupción cada 60 seg,
 * donde se guardarán los datos, y se iniciará la polarización de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - 
 */
#ifndef TIMER_COUNTER_H
#define TIMER_COUNTER_H

#include "interrupts.h"
#define DEBUG_TC

extern bool detect_TC;
extern bool detect1_TC;

void setupTC2(uint8_t segundos);        // Configuración de interrupción por tiempo
void TC2_Handler(void);                 // Bandera 
void disableTC2(void);                  // NVIC_DisableIRQ()
void enableTC2(void);                   // NVIC_EnableIRQ()

void setupTC0(void);        
void TC0_Handler(void);     
void setupTC4(void);        // Para pulsos
void TC4_Handler(void);     // Calculo del pulso

#endif 

// PA16-17 para 32 bits