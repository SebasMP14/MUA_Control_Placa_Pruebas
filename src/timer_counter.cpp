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

  // Configurar la comparación para generar una interrupción
  TC2->COUNT32.CC[1].reg = (120000000 * 10 / 1024); // CC_Value = clk_frec * Tiempo_deseado / prescaler
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

void setupTC_Pulse1(void) {
  GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Usar GCLK0

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST; // Resetear TC4
  while (TC4->COUNT16.SYNCBUSY.bit.SWRST);

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16; // Ajustar el prescaler
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Modo de 16 bits

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; // Habilitación
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE);
}