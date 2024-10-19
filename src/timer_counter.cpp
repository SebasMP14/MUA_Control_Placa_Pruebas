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
#include "Arduino.h"
#include "timer_counter.h"
#include "hardware_pins.h"

bool detect_TC = false;
bool detect1_TC = false;
unsigned long pulse_Width = 0x00;

/************************************************************************************************************
 * @fn      setupTC()
 * @brief   Configuración de timer counter de 32 bits, utiliza 2 registros (TC0 y TC1, o TC2 y TC3)
 * @param   NONE
 * @return  NONE
 */
void setupTC2(void) {
  // Habilitar el reloj para TC2
  GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Usa el generador de reloj 0
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Habilitar reloj para TC3 también (parte del contador de 32 bits)
  
  TC2->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC2
  while (TC2->COUNT32.SYNCBUSY.bit.SWRST);  // Esperar hasta que se complete el reseteo
  TC3->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC3
  while (TC3->COUNT32.SYNCBUSY.bit.SWRST);  // 

  // Configurar el prescaler y modo
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; // Prescaler de 1024
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32; // Modo de 32 bits

  /* Configurar la comparación para generar una interrupción.
  El orden de los factores, altera el manejo de la variable. */
  TC2->COUNT32.CC[1].reg = ((120000000 / 1024) * 30); // CC_Value = clk_frec * Tiempo_deseado / prescaler
  while (TC2->COUNT32.SYNCBUSY.bit.CC1);

  // Habilitar interrupciones por comparación
  TC2->COUNT32.INTENSET.reg = TC_INTENSET_MC1; // Habilitar interrupción de desbordamiento

  // Habilitar el TC2
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC2->COUNT32.SYNCBUSY.bit.ENABLE); // Esperar a que TC2 se habilite

  NVIC_EnableIRQ(TC2_IRQn); // Habilitar la interrupción en el NVIC
}

/************************************************************************************************************
 * @fn      TC_Handler()
 * @brief   Manejo de la interrupción, se resetea el overflow y el registro de conteo del tiempo.
 * @param   NONE
 * @return  NONE
 */
void TC2_Handler(void) {
  if (TC2->COUNT32.INTFLAG.bit.MC1) { // Si ocurre un overflow
    TC2->COUNT32.INTFLAG.bit.MC1 = 1; // Limpiar la bandera de interrupción
    TC2->COUNT32.COUNT.reg = 0x0; // Se resetea el registro
    detect_TC = true;
    // digitalWrite(SCL_Sensor, !digitalRead(SCL_Sensor)); // Alternar el estado del LED
    #ifdef DEBUG_TC
    Serial.println("DEBUG (TC2_Handler): Interrupción TC.");
    #endif
  }
}

void setupTC4(void) {
  // Habilitar el bus para el TC4
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4;

  // Configurar el TC4 para usar el clock gen (usar GCLK0 a 120MHz por ejemplo)
  GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
  while (GCLK->PCHCTRL[TC4_GCLK_ID].bit.CHEN == 0); // Esperar hasta que esté listo

  // Configuración del pin PB08 como entrada para la captura (TC4_CH0)
  // PORT->Group[PORTB].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;       // Habilitar el multiplexer
  // PORT->Group[PORTB].PMUX[8 >> 1].reg &= ~PORT_PMUX_PMUXE_Msk;  // Limpiar los bits anteriores de PMUXE
  // PORT->Group[PORTB].PMUX[8 >> 1].reg |= PORT_PMUX_PMUXE(0x05); // TC4_CH0 en PB08

  // Configuración del CTRLA (modo de 16 bits, prescaler 8 y habilitar captura CH0)
  // TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_CAPTEN0;
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_CAPTEN0 | TC_CTRLA_COPEN0;

  // Configuración de la captura de eventos
  // TC4->COUNT16.EVCTRL.reg = TC_EVCTRL_EVACT_PW | TC_EVCTRL_TCEI;  // Capture pulse-width
  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;  // Habilitar interrupción en capture match
  TC4->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI;
  NVIC_EnableIRQ(TC4_IRQn);  // Habilitar la interrupción global del TC4



  TC4->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); // Esperar hasta que esté sincronizado

  #ifdef DEBUG_TC
  Serial.println("DEBUG (setupTC4): Setup exitoso?");
  #endif

}

void TC4_Handler(void) {
  if (TC4->COUNT16.INTFLAG.bit.MC0) {
    pulse_Width = TC4->COUNT16.CC[0].reg;  // Leer el valor capturado en el canal 0
    
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;  // Limpiar la bandera de interrupción
    #ifdef DEBUG_TC
    Serial.println("DEBUG (TC4_Handler): Pulse width.");
    #endif
    detect1_TC = true;
  }
}