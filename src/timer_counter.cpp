#include "Arduino.h"
#include "timer_counter.h"
#include "hardware_pins.h"

void setupTC2() {
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
  TC2->COUNT32.CC[0].reg = (120000000 * 10 / 1024); // CC_Value = clk_frec * Tiempo_deseado / prescaler
  while (TC2->COUNT32.SYNCBUSY.bit.CC0);

  // Habilitar interrupciones por comparación
  TC2->COUNT32.INTENSET.reg = TC_INTENSET_MC0; // Habilitar interrupción de desbordamiento

  // Habilitar el TC2
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC2->COUNT32.SYNCBUSY.bit.ENABLE); // Esperar a que TC2 se habilite

  // Habilitar la interrupción en el NVIC
  NVIC_EnableIRQ(TC2_IRQn);
}

// Manejador de interrupciones de TC2
void TC2_Handler() {
  if (TC2->COUNT32.INTFLAG.bit.MC0) { // Si ocurre un overflow
    TC2->COUNT32.INTFLAG.bit.MC0 = 1; // Limpiar la bandera de interrupción
    TC2->COUNT32.COUNT.reg = 0x0; // Se resetea el registro
    digitalWrite(SCL_Sensor, !digitalRead(SCL_Sensor)); // Alternar el estado del LED
    Serial.println("Interrupción.");
  }

}

