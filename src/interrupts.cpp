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
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile uint16_t aux = 0;

void handlePulse1_R(void) {
	pulse_count1++;
	detect1 = true;
}

void handlePulse1_F(void) {
	pulse_count1++;
	detect1 = true;
}

void handlePulse2_R(void) {
	pulse_count2++;
	detect2 = true;
}

void activeInterrupt(void) {
	attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1_R, RISING);
	// attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1_F, FALLING);
  // attachInterrupt(digitalPinToInterrupt(PULSE_2), handlePulse1_R, RISING);
}

void desactiveInterrupt(void) {
	detachInterrupt(digitalPinToInterrupt(PULSE_1));
	// detachInterrupt(digitalPinToInterrupt(PULSE_2));
}