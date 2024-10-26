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
 * - Ajustar el tiempo de interrupción de TC2 de acuerdo a la variación de temperatura.
 *   Esto debido a la tasa de variación del Vbr.
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
    Serial.println("DEBUG (TC2_Handler) -> Interrupción TC2.");
    #endif
  }
}

/************************************************************************************************************
 * @fn      setupTC()
 * @brief   Configuración de timer counter de 16 bits 
 * @param   NONE
 * @return  NONE
 */
void setupTC4(void) {
  // Habilitar el reloj para TC4
  GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Usa el generador de reloj 0
  
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC4
  while (TC4->COUNT16.SYNCBUSY.bit.SWRST);  // Esperar hasta que se complete el reseteo
  
  // Configurar el prescaler y modo
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; // Prescaler de 1024
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Modo de 16 bits

  /* Configurar la comparación para generar una interrupción.
  El orden de los factores, altera el manejo de la variable. */
  // TC4->COUNT16.CC[0].reg = 0xFFFF; // CC_Value = clk_frec * Tiempo_deseado / prescaler
  // while (TC4->COUNT16.SYNCBUSY.bit.CC0);

  // // Habilitar interrupciones por comparación
  // TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0; // Habilitar interrupción de desbordamiento

  TC4->COUNT16.CTRLBSET.reg |= TC_CTRLBSET_LUPD;    // Habilitar la captura automática en eventos de flanco
  TC4->COUNT16.CTRLBSET.reg |= TC_CTRLBSET_DIR; 
  TC4->COUNT16.CC[0].reg = 0;    // Canal de captura 0 (flanco de subida)
  TC4->COUNT16.CC[1].reg = 0;    // Canal de captura 1 (flanco de bajada)
  while (TC4->COUNT16.SYNCBUSY.bit.CC0 || TC4->COUNT16.SYNCBUSY.bit.CC1);  // Sincronizar
  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0|TC_INTENSET_MC1;


  // Habilitar el TC4
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); // Esperar a que TC4 se habilite

  NVIC_EnableIRQ(TC4_IRQn); // Habilitar la interrupción en el NVIC
}

/************************************************************************************************************
 * @fn      TC_Handler()
 * @brief   Manejo de la interrupción, se resetea el overflow y el registro de conteo del tiempo.
 * @param   NONE
 * @return  NONE
 */
void TC4_Handler(void) {
  // if (TC4->COUNT16.INTFLAG.bit.MC0) { // Si ocurre un overflow
  //   TC4->COUNT16.INTFLAG.bit.MC0 = 1; // Limpiar la bandera de interrupción
  //   TC4->COUNT16.COUNT.reg = 0x0; // Se resetea el registro
  //   #ifdef DEBUG_TC
  //   Serial.println("DEBUG (TC4_Handler) -> Interrupción TC4.");
  //   #endif
  // }

  static uint16_t rising_edge_time = 0;
  static uint16_t falling_edge_time = 0;
  
  // Captura en flanco de subida
  if (TC4->COUNT16.INTFLAG.bit.MC0 == 0) {
    rising_edge_time = TC4->COUNT16.CC[0].reg;  // Capturar el tiempo en flanco de subida
    TC4->COUNT16.INTFLAG.bit.MC0 = 1;  // Limpiar la bandera de captura de flanco de subida
    return;
  }

  // Captura en flanco de bajada
  if (TC4->COUNT16.INTFLAG.bit.MC1) {
    TC4->COUNT16.INTFLAG.bit.MC1 = 1;  // Limpiar la bandera de captura de flanco de bajada
    falling_edge_time = TC4->COUNT16.CC[1].reg;  // Capturar el tiempo en flanco de bajada
  }
    
    // Calcular el ancho del pulso
    pulse_width = falling_edge_time - rising_edge_time;
}