/**
 * timer_counter.cpp
 *  Configuración y manejo de interrupción del timer counter (32bits). Se espera una interrupción cada 60 seg,
 * donde se guardarán los datos, y se iniciará la polarización de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Implementar la escritura de datos en memoria flash y el algoritmo de polarización
 */
#ifndef TIMER_COUNTER_H
#define TIMER_COUNTER_H

#include "interrupts.h"
#define DEBUG_TC

extern bool detect_TC;
extern bool detect1_TC;

void setupTC0(void);        
void setupTC2(void);        // Para algoritmo de polarización
void setupTC4(void);        // Para pulsos
void TC0_Handler(void);     
void TC2_Handler(void);     // Aplicación del algoritmo
void TC4_Handler(void);     // Calculo del pulso

#endif 

// PA16-17 para 32 bits