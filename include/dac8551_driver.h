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
uint16_t VDAC_command(float max_vout, float voltage);
float out_voltage(float max_vout, uint16_t Vcommand);

#endif