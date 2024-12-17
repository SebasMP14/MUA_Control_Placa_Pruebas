/**
 * flash_driver.h
 * Funciones de control de la memoria flash (W25Q128JV_SM) para la misión M.U.A. de FIUNA 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * - Ing. Lucas Cho 
 * 
 * 
 * 
 * TODO:
 * - Escribir mensajes al OBC
 * - read_all() debe transferir los datos por UART y luego ejecutar erase_all()
 */

#ifndef FLASH_DRIVER_H
#define FLASH_DRIVER_H

#include <stdint.h>  
#include <stdbool.h>

#include <Adafruit_SPIFlash.h>
#include "hardware_pins.h"

extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash Flash_QSPI;

#define DEBUG_FLASH
// #define DEBUG_FLASH_INFO
#define MAX_ITERATIONS            10
#define SAVED_ADDRESS_SECTOR      4095                            /* Numero de sector utilzado para almacenar 4 bytes que indican la ultima posicion de memoria donde se escribieron los datos */
#define SAVED_ADDRESS_SECTOR_DIR  (SAVED_ADDRESS_SECTOR * 4096)   /* Direccion del sector utilizado para almacenar los 4 bytes que indican la última posición de memoria donde se escribieron los datos */
#define PAGE_SIZE                 256                             /* De la memoria flash */ 
#define ADDRESS_SIZE              4                               /* Cada direccion de memoria tiene 4 bytes */
#define SECTOR_SIZE               4096
#define EXT_FLASH_SIZE            (16UL * 1024 * 1024)            /* 16 MB o 128 Mb */
#define LAST_PAGE_ADDR            (EXT_FLASH_SIZE - PAGE_SIZE)    /* Contiene la última posición escrita */
#define LAST_ADDRESS_SENDED       SAVED_ADDRESS_SECTOR_DIR + ADDRESS_SIZE // 


bool start_flash(void);
bool write_mem(uint8_t *buffer, uint32_t len);
bool get_address(uint32_t *write_address);
void read_all(void);
void read_until(uint8_t *data, uint32_t data_length);
uint8_t read_OPstate(void);
bool erase_all(void);
bool update_address(uint32_t *adress);

#endif