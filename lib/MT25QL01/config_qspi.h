#ifndef CONFIG_QSPI_H
#define CONFIG_QSPI_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "wiring_private.h"
#include "hardware_pins.h"

// MT25QL01GBBB COMMANDS
enum {                                                                
  SFLASH_CMD_READ =               0x03,      // Single Read                         // si
  SFLASH_CMD_FAST_READ =          0x0B, // Fast Read                           // si
  SFLASH_CMD_QUAD_READ =          0x6B, // 1 line address, 4 line data         // si

  SFLASH_CMD_READ_JEDEC_ID =      0x9f,                                    // si

  SFLASH_CMD_PAGE_PROGRAM =       0x02,                                     // si
  SFLASH_CMD_QUAD_PAGE_PROGRAM =  0x32, // 1 line address, 4 line data // si

  SFLASH_CMD_READ_STATUS =        0x05,                                      // si

  SFLASH_CMD_QUAD_EN =            0x35,                                     // si pero no

  SFLASH_CMD_WRITE_STATUS =       0x01,                                     // si
  SFLASH_CMD_WRITE_STATUS2 =      0x31,                                    // 

  SFLASH_CMD_ENABLE_RESET =       0x66,                                     // si
  SFLASH_CMD_RESET =              0x99,                                            // si

  SFLASH_CMD_WRITE_ENABLE =       0x06,                                     // si
  SFLASH_CMD_WRITE_DISABLE =      0x04,                                    // si

  SFLASH_CMD_ERASE_PAGE =         0x81,                                       // 
  SFLASH_CMD_ERASE_SECTOR =       0x20,                                     // si
  SFLASH_CMD_ERASE_BLOCK =        0xD8,                                      // si
  SFLASH_CMD_ERASE_CHIP =         0xC7,                                       // 0xc4

  SFLASH_CMD_4_BYTE_ADDR =        0xB7,                                      // si
  SFLASH_CMD_3_BYTE_ADDR =        0xE9,                                      // si

};
/*
#define CMD_WRITE_ENABLE                    0x06
#define CMD_STATUSREG_READ                  0x05
#define CMD_READ_NONVOL_CFG_REG             0xB5
#define CMD_WRITE_NONVOL_CFG_REG            0xB1
#define CMD_READ_ID2                        0x9F
#define CMD_RESET_ENABLE                    0x66
#define CMD_RESET_MEMORY                    0x99
#define CMD_READ_VOL_CFG_REG                0x85
#define CMD_WRITE_VOL_CFG_REG               0x81
#define CMD_DIE_ERASE                       0xC4
#define CMD_EN_4_BYTE_ADDR_MODE             0xB7
#define CMD_SUBSECTOR_ERASE_4_BYTE          0x21
#define CMD_QUAD_IO_READ_4_BYTE             0xEC
#define CMD_EXT_QUAD_IN_FAST_PROG_4_BYTE    0x3E
*/

static void samd_peripherals_disable_and_clear_cache(void);
static void samd_peripherals_enable_cache(void);
void begin(void);
void end(void);
bool runCommand(uint8_t command);
bool readCommand(uint8_t command, uint8_t *response, uint32_t len);
bool writeCommand(uint8_t command, uint8_t const *data, uint32_t len);
bool eraseCommand(uint8_t command, uint32_t address);
bool readMemory(uint32_t addr, uint8_t *data, uint32_t len);
bool writeMemory(uint32_t addr, uint8_t const *data, uint32_t len);
void setClockSpeed(uint32_t clock_hz, uint32_t read_hz);
static void _run_instruction(uint8_t command, uint32_t iframe, uint32_t addr, uint8_t *buffer, uint32_t size);
bool runQCommand(uint8_t command);
bool readQCommand(uint8_t command, uint8_t *response, uint32_t len);
bool writeQCommand(uint8_t command, uint8_t const *data, uint32_t len);
bool eraseQCommand(uint8_t command, uint32_t address);

// Turn off cache and invalidate all data in it.
static void samd_peripherals_disable_and_clear_cache(void) {
  CMCC->CTRL.bit.CEN = 0;
  while (CMCC->SR.bit.CSTS) {
  }
  CMCC->MAINT0.bit.INVALL = 1;
}

// Enable cache
static void samd_peripherals_enable_cache(void) { 
  CMCC->CTRL.bit.CEN = 1;
}

void begin(void) {
  MCLK->APBCMASK.bit.QSPI_ = true;
  MCLK->AHBMASK.bit.QSPI_ = true;
  MCLK->AHBMASK.bit.QSPI_2X_ = false; // Only true if we are doing DDR.

  QSPI->CTRLA.bit.SWRST = 1;

  // set all pins to QSPI periph
  pinPeripheral(QSPI_SCK, PIO_COM);
  pinPeripheral(QSPI_CS,  PIO_COM);
  pinPeripheral(QSPI_D0,  PIO_COM);
  pinPeripheral(QSPI_D1,  PIO_COM);
  pinPeripheral(QSPI_D2,  PIO_COM);
  pinPeripheral(QSPI_D3,  PIO_COM);

  QSPI->BAUD.reg =
      QSPI_BAUD_BAUD(VARIANT_MCK / 4000000UL); // start with low 4Mhz, Mode 0
  QSPI->CTRLB.reg = QSPI_CTRLB_MODE_MEMORY | QSPI_CTRLB_DATALEN_8BITS |
                      QSPI_CTRLB_CSMODE_LASTXFER;

  QSPI->CTRLA.bit.ENABLE = 1;
}

void end(void) {
  QSPI->CTRLA.bit.ENABLE = 0;

  MCLK->APBCMASK.bit.QSPI_ = false;
  MCLK->AHBMASK.bit.QSPI_ = false;
  MCLK->AHBMASK.bit.QSPI_2X_ = false;
}

bool runCommand(uint8_t command) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_SINGLE_BIT_SPI |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_READ | QSPI_INSTRFRAME_INSTREN;

  _run_instruction(command, iframe, 0, NULL, 0);
  return true;
}


bool readCommand(uint8_t command, uint8_t *response, uint32_t len) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_SINGLE_BIT_SPI |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_READ | QSPI_INSTRFRAME_INSTREN |
                    QSPI_INSTRFRAME_DATAEN;

samd_peripherals_disable_and_clear_cache();
_run_instruction(command, iframe, 0, response, len);
samd_peripherals_enable_cache();

return true;
}

bool writeCommand(uint8_t command, uint8_t const *data, uint32_t len) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_SINGLE_BIT_SPI |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_WRITE | QSPI_INSTRFRAME_INSTREN |
                    (data != NULL ? QSPI_INSTRFRAME_DATAEN : 0);

  samd_peripherals_disable_and_clear_cache();
  _run_instruction(command, iframe, 0, (uint8_t *)data, len);
  samd_peripherals_enable_cache();

  return true;
}

// Sector Erase
bool eraseCommand(uint8_t command, uint32_t address) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_SINGLE_BIT_SPI |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_WRITE | QSPI_INSTRFRAME_INSTREN |
                    QSPI_INSTRFRAME_ADDREN;

  _run_instruction(command, iframe, address, NULL, 0);
  return true;
}

bool readMemory(uint32_t addr, uint8_t *data, uint32_t len) {
  // Command 0x6B 1 line address, 4 line Data
  // Quad output mode, read memory type
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT |
  QSPI_INSTRFRAME_ADDRLEN_24BITS |
  QSPI_INSTRFRAME_TFRTYPE_READMEMORY |
  QSPI_INSTRFRAME_INSTREN | QSPI_INSTRFRAME_ADDREN |
  QSPI_INSTRFRAME_DATAEN | QSPI_INSTRFRAME_DUMMYLEN(8);

  samd_peripherals_disable_and_clear_cache();
  _run_instruction(SFLASH_CMD_QUAD_READ, iframe, addr, data, len);
  samd_peripherals_enable_cache();

  return true;
}

bool writeMemory(uint32_t addr, uint8_t const *data, uint32_t len) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT | QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_WRITEMEMORY | QSPI_INSTRFRAME_INSTREN |
                    QSPI_INSTRFRAME_ADDREN | QSPI_INSTRFRAME_DATAEN;

  samd_peripherals_disable_and_clear_cache();
  _run_instruction(SFLASH_CMD_QUAD_PAGE_PROGRAM, iframe, addr, (uint8_t *)data, len);
  samd_peripherals_enable_cache();

  return true;
}

void setClockSpeed(uint32_t clock_hz, uint32_t read_hz) {
  (void)read_hz;
  QSPI->BAUD.bit.BAUD = VARIANT_MCK / clock_hz;
}

static void _run_instruction(uint8_t command, uint32_t iframe, uint32_t addr, uint8_t *buffer, uint32_t size) {
  if (command == SFLASH_CMD_ERASE_SECTOR || command == SFLASH_CMD_ERASE_BLOCK) {
    QSPI->INSTRADDR.reg = addr;
  }

  QSPI->INSTRCTRL.bit.INSTR = command;
  QSPI->INSTRFRAME.reg = iframe;

  // Dummy read of INSTRFRAME needed to synchronize.
  // See Instruction Transmission Flow Diagram, figure 37.9, page 995 "<3"
  // and Example 4, page 998, section 37.6.8.5.
  volatile uint32_t dummy = QSPI->INSTRFRAME.reg;
  (void)dummy;

  if (buffer && size) {
    uint8_t *qspi_mem = (uint8_t *)(QSPI_AHB + addr);
    uint32_t const tfr_type = iframe & QSPI_INSTRFRAME_TFRTYPE_Msk;

    if ((tfr_type == QSPI_INSTRFRAME_TFRTYPE_READ) || (tfr_type == QSPI_INSTRFRAME_TFRTYPE_READMEMORY)) {
      memcpy(buffer, qspi_mem, size);
    } else {
      memcpy(qspi_mem, buffer, size);
    }
  }

  QSPI->CTRLA.reg = QSPI_CTRLA_ENABLE | QSPI_CTRLA_LASTXFER;

  while (!QSPI->INTFLAG.bit.INSTREND) {
    yield();
  }
  QSPI->INTFLAG.bit.INSTREND = 1;
}

/////////////////////////////////////////////////////////////////////////////
////////////////////////////// Se agregan comandos QSPI /////////////////////

bool runQCommand(uint8_t command) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_READ | QSPI_INSTRFRAME_INSTREN;

  _run_instruction(command, iframe, 0, NULL, 0);
  return true;
}

bool readQCommand(uint8_t command, uint8_t *response, uint32_t len) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_READ | QSPI_INSTRFRAME_INSTREN |
                    QSPI_INSTRFRAME_DATAEN;

  samd_peripherals_disable_and_clear_cache();
  _run_instruction(command, iframe, 0, response, len);
  samd_peripherals_enable_cache();

  return true;
}

bool writeQCommand(uint8_t command, uint8_t const *data, uint32_t len) {
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_WRITE | QSPI_INSTRFRAME_INSTREN |
                    (data != NULL ? QSPI_INSTRFRAME_DATAEN : 0);

  samd_peripherals_disable_and_clear_cache();
  _run_instruction(command, iframe, 0, (uint8_t *)data, len);
  samd_peripherals_enable_cache();

  return true;
}

bool eraseQCommand(uint8_t command, uint32_t address) {
  // Sector Erase
  uint32_t iframe = QSPI_INSTRFRAME_WIDTH_QUAD_OUTPUT |
                    QSPI_INSTRFRAME_ADDRLEN_24BITS |
                    QSPI_INSTRFRAME_TFRTYPE_WRITE | QSPI_INSTRFRAME_INSTREN |
                    QSPI_INSTRFRAME_ADDREN;

  _run_instruction(command, iframe, address, NULL, 0);
  return true;
}
#endif