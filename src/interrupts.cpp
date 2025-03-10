/**
 * interrupts.cpp
 * Manejo de interrupciones para los pulsos digitales provenientes del acondicionamiento de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - Si se presenta un overflow, manejarlo. Se puede guardar todo en memoria y liberar las variables o ...
 */
#include "interrupts.h"

volatile uint16_t pulse_count1 = 0;  // Contador de pulsos
volatile uint16_t pulse_count2 = 0;
volatile bool detect1 = false;
volatile bool detect2 = false;

void handlePulse1_R(void) {
	pulse_count1++;
	detect1 = true;
}

void handlePulse2_R(void) {
	pulse_count2++;
	detect2 = true;
}

void activeInterrupt1(void) {
	attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1_R, RISING);
}
void activeInterrupt2(void) {
  attachInterrupt(digitalPinToInterrupt(PULSE_2), handlePulse2_R, RISING);
}

void desactiveInterrupt1(void) {
	detachInterrupt(digitalPinToInterrupt(PULSE_1));
}
void desactiveInterrupt2(void) {
	detachInterrupt(digitalPinToInterrupt(PULSE_2));
}