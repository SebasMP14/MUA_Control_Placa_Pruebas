/**
 * @file timer_counter.cpp
 *  Configuración y manejo de interrupción del timer counter (32bits). Se espera una interrupción cada 60 seg,
 * donde se guardarán los datos, y se iniciará la polarización de los SiPMs.
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - 
 * - Ajustar el tiempo de interrupción de TC2 de acuerdo a la variación de temperatura.
 *   Esto debido a la tasa de variación del Vbr.
 */
#include "Arduino.h"
#include "timer_counter.h"
#include "hardware_pins.h"

bool detect_TC = false;
bool detect1_TC = false;

/************************************************************************************************************
 * @fn      setupTC()
 * @brief   Configuración de timer counter de 32 bits, utiliza 2 registros (TC0 y TC1, o TC2 y TC3)
 * @param   NONE
 * @return  NONE
 */
void setupTC2(uint8_t segundos) {
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
  TC2->COUNT32.CC[1].reg = ((120000000 / 1024) * segundos); // CC_Value = clk_frec * Tiempo_deseado / prescaler
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
    #ifdef DEBUG_TC
    Serial.println("DEBUG (TC2_Handler) -> Interrupción TC2.");
    #endif
  }
}

/************************************************************************************************************
 * @fn      disableTC2()
 * @brief   Desabilitación del TC2.
 * @param   NONE
 * @return  NONE
 */
void disableTC2(void) {
  NVIC_DisableIRQ(TC2_IRQn);
  TC2->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;  // Deshabilitar el TC2
  while (TC2->COUNT32.SYNCBUSY.bit.ENABLE);  // Esperar a que se deshabilite
  TC2->COUNT32.INTFLAG.reg = TC_INTFLAG_MC1;  // Limpiar bandera de interrupción
}

/************************************************************************************************************
 * @fn      enableTC2()
 * @brief   Habilitación del TC2.
 * @param   NONE
 * @return  NONE
 */
void enableTC2(void) {
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;  // Habilitar el TC2
  while (TC2->COUNT32.SYNCBUSY.bit.ENABLE);   // Esperar a que TC2 se habilite
  TC2->COUNT32.INTFLAG.reg = TC_INTFLAG_MC1;  // Limpiar bandera de interrupción
  NVIC_EnableIRQ(TC2_IRQn);
}