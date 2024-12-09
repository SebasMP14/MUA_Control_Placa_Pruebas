/**
 * max1932_driver.h
 * Funciones de control del max1932, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Command de la función write debería ser la tensión de salida deseada
 */
#ifndef MAX1932_DRIVER_H
#define MAX1932_DRIVER_H

#include <SPI.h>
#include "hardware_pins.h"

#define DEBUG_MAX
#define SPI_CLK_Speed 1000000
#define SPI_CS_delay 2 * SPI_CLK_Speed / 1000000

void start_max1932(void);
bool write_max_reg(uint8_t command);
uint8_t VMax_command(float valor);
float HexMax_command(uint8_t valor);

#endif