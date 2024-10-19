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
volatile unsigned long pulse_start = 0;
volatile unsigned long pulse_width = 0;

void handlePulse1(void) {
	pulse_count1++;
	detect1 = true;
	// if (digitalRead(PB08) == HIGH) {
	// 	// Capturar el tiempo del flanco ascendente
	// 	while (TC4->COUNT16.SYNCBUSY.bit.COUNT);
	// 	pulse_start = TC4->COUNT16.COUNT.reg;
	// } else {
	// 	// Capturar el tiempo del flanco descendente
	// 	while (TC4->COUNT16.SYNCBUSY.bit.COUNT);

	// 	// Calcular el ancho del pulso
	// 	pulse_width = TC4->COUNT16.COUNT.reg - pulse_start;
	// 	// detect1 = true;
	// }
}

void handlePulse2(void) {
	pulse_count2++;
	detect2 = true;
}