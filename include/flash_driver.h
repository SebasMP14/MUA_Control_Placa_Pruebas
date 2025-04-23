/**
 * flash_driver.h
 * Funciones de control de la memoria flash (MT25QL01GBBB) para la misión M.U.A. de FIUNA 
 *  La memoria flash se separa en dos secciones de almacenamiento:
 * - La sección de DATOS almacena las tramas generadas en loopCount y va desde el sector cero hasta 
 * el penúltimo sector de la memoria flash
 * - La sección SYSINFO almacena variables importantes de operación del sistema de control y abarca sólo
 * el último sector de la memoria flash
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * - Ing. Lucas Cho 
 * 
 * TODO:
 * - 
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
#define MAX_ITERATIONS            5
#define SAVED_SYSINFO_SECTOR      4095                            /* Numero de sector utilzado para almacenar 4 bytes que indican la ultima posicion de memoria donde se escribieron los datos */
#define SAVED_ADDRESS_SECTOR_DIR  (SAVED_SYSINFO_SECTOR * 4096)   /* Direccion del sector utilizado para almacenar los 4 bytes que indican la última posición de memoria donde se escribieron los datos */
#define ADDRESS_SIZE              4                               /* Cada direccion de memoria tiene 4 bytes */
#define SECTOR_SIZE               4096
#define EXT_FLASH_SIZE            (16UL * 1024 * 1024)            // CAMBIAR a 128MB
#define LAST_ADDRESS_SENT_DIR     SAVED_ADDRESS_SECTOR_DIR + ADDRESS_SIZE // 
#define ADDRESS_OP_STATE_DIR      LAST_ADDRESS_SENT_DIR + ADDRESS_SIZE * 2 - 1
#define SIZE_INFO                 12                              // Bytes de variables/info de sistema
#define SAVED_INDEX               0
#define SENT_INDEX                4
#define OP_STATE_INDEX            11
// #define PAGE_SIZE                 256                             /* De la memoria flash */ 
// #define LAST_PAGE_ADDR            (EXT_FLASH_SIZE - PAGE_SIZE)    /* Contiene la última posición escrita */


bool start_flash(void);
bool get_address(uint32_t *write_address);
bool get_OPstate(uint8_t *read);
bool get_SENT_DATAaddress(uint32_t *sent_address);
bool write_mem(uint8_t *buffer, uint32_t len);
bool update_address(uint32_t *adress);
void read_all(void);
void read_until(uint8_t *data, uint32_t data_length);
bool read(uint8_t *data, uint32_t data_length, uint32_t since); 
bool write_DATAinfo(uint8_t *buffer, uint32_t len, uint16_t index);
bool write_OPstate(uint8_t state);
bool write_SENT_DATAaddress(uint32_t *address);
bool erase_all(void);
bool erase_debug(void);

#endif