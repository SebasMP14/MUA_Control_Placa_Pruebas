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

#define DEBUG_TC

extern bool detect_TC;

void setupTC0(void);        // Interrupción para algoritmo de polarización
void setupTC2(void);        
void TC0_Handler(void);     // Aplicación del algoritmo
void TC2_Handler(void);
void setupTC_Pulse1(void);

#endif 

// PA16-17 para 32 bits