#include "interrupts.h"

volatile uint32_t pulse_count1 = 0;  // Contador de pulsos
volatile bool detect = false; 

void handlePulse1() {
    pulse_count1++;  // Incrementar el contador de pulsos
    detect = true;   // Indicar que se ha detectado un pulso
}