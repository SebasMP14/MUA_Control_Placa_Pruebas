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
  SPI.begin();
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