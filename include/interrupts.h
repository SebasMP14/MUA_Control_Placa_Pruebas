#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <stdint.h>  
#include <stdbool.h>

// Variables globales
extern volatile uint32_t pulse_count1;  // Contador de pulsos
extern volatile bool detect;  // Flag para indicar detección de pulso

void handlePulse1();

#endif