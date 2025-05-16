/**
 * dac8551_driver.cpp
 * Funciones de control del dac8551, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2025> (github)
 * 
 * TODO:
 * - 
 */

#include "dac8551_driver.h"
#include <Arduino.h>

void start_dac8551(uint8_t chip_select) {
  pinMode(chip_select, OUTPUT);
  digitalWrite(chip_select, HIGH);
  // SPI.begin();
}

void write_dac8551_reg(uint16_t command, uint8_t chip_select) {
  SPI.beginTransaction(SPISettings(DAC_CLK_SPEED, MSBFIRST, SPI_MODE1));
  digitalWrite(chip_select, LOW);  // selección
  SPI.transfer(0x00);
  SPI.transfer16(command);
  digitalWrite(chip_select, HIGH);
  SPI.endTransaction();
}

void end_dac8551(uint8_t chip_select) {
  SPI.beginTransaction(SPISettings(DAC_CLK_SPEED, MSBFIRST, SPI_MODE1)); 
  digitalWrite(chip_select, LOW);  // selección
  SPI.transfer(0x11);
  SPI.transfer16(0x0000);
  digitalWrite(chip_select, HIGH);
  SPI.endTransaction();
}

/************************************************************************************************************
 * @fn      VMax_command
 * @brief   Conversión de voltaje a commando bin para el MAX
 * @param   valor: Es el voltaje objetivo en la salida del MAX
 * @return  comando en binario
 */
uint16_t VDAC_command(float max_vout, float voltage) {
  return static_cast<uint16_t>((max_vout - voltage) * 32767 / 12); // 36 para la Placa de deteccion anterior
}

float out_voltage(float max_vout, uint16_t Vcommand) {
  return static_cast<float>(max_vout - (Vcommand * 12 / 32767.0)); // 36 para la Placa de deteccion anterior
}