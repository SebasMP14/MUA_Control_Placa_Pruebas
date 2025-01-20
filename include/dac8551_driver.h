/**
 * dac8551_driver.h
 * Funciones de control del dac8551, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2025> (github)
 * 
 * TODO:
 * - 
 */

#ifndef DAC8551_DRIVER_H
#define DAC8551_DRIVER_H

#include <SPI.h>
#include "hardware_pins.h"

#define DEBUG_DAC
#define DAC_CLK_SPEED 1000000

void start_dac8551(uint8_t chip_select);
void write_dac8551_reg(uint16_t command, uint8_t chip_select);
void end_dac8551(uint8_t chip_select);

#endif