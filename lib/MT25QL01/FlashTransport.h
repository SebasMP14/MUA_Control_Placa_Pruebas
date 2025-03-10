#ifndef FLASHTRANSPORT_H
#define FLASHTRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware_pins.h"

// #define PLACA_PRUEBAS
// #define PLACA_FINAL

enum {                                                                // MT25QL01GBBB
  SFLASH_CMD_READ = 0x03,      // Single Read                         // si
  SFLASH_CMD_FAST_READ = 0x0B, // Fast Read                           // si
  SFLASH_CMD_QUAD_READ = 0x6B, // 1 line address, 4 line data         // si

  SFLASH_CMD_READ_JEDEC_ID = 0x9f,                                    // si

  SFLASH_CMD_PAGE_PROGRAM = 0x02,                                     // si
  SFLASH_CMD_QUAD_PAGE_PROGRAM = 0x32, // 1 line address, 4 line data // si

  SFLASH_CMD_READ_STATUS = 0x05,                                      // si
  SFLASH_CMD_READ_STATUS2 = 0x35,                                     // si pero no

  SFLASH_CMD_WRITE_STATUS = 0x01,                                     // si
  SFLASH_CMD_WRITE_STATUS2 = 0x31,                                    // 

  SFLASH_CMD_ENABLE_RESET = 0x66,                                     // si
  SFLASH_CMD_RESET = 0x99,                                            // si

  SFLASH_CMD_WRITE_ENABLE = 0x06,                                     // si
  SFLASH_CMD_WRITE_DISABLE = 0x04,                                    // si

  SFLASH_CMD_ERASE_PAGE = 0x81,                                       // 
  SFLASH_CMD_ERASE_SECTOR = 0x20,                                     // si
  SFLASH_CMD_ERASE_BLOCK = 0xD8,                                      // si
  SFLASH_CMD_ERASE_CHIP = 0xC7,                                       // 0xc4

  SFLASH_CMD_4_BYTE_ADDR = 0xB7,                                      // si
  SFLASH_CMD_3_BYTE_ADDR = 0xE9,                                      // si

};

/// Constant that is (mostly) true to all external flash devices
enum {
  SFLASH_BLOCK_SIZE = 64 * 1024UL,
  SFLASH_SECTOR_SIZE = 4 * 1024,
  SFLASH_PAGE_SIZE = 256,
};

class FlashTransport {
public:
  virtual void begin(void) = 0;
  virtual void end(void) = 0;

  virtual bool supportQuadMode(void) = 0;
  virtual void setClockSpeed(uint32_t write_hz, uint32_t read_hz) = 0;

  virtual bool runCommand(uint8_t command) = 0;
  virtual bool readCommand(uint8_t command, uint8_t *response, uint32_t len) = 0;
  virtual bool writeCommand(uint8_t command, uint8_t const *data, uint32_t len) = 0;
  virtual bool eraseCommand(uint8_t command, uint32_t address) = 0;
  virtual bool readMemory(uint32_t addr, uint8_t *buffer, uint32_t len) = 0;
  virtual bool writeMemory(uint32_t addr, uint8_t const *data, uint32_t len) = 0;
  void setAddressLength(uint8_t addr_len) { _addr_len = addr_len; }
  void setReadCommand(uint8_t cmd_read) { _cmd_read = cmd_read; }

  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Se agregan comandos QSPI /////////////////////
  virtual bool runQCommand(uint8_t command) = 0;
  virtual bool readQCommand(uint8_t command, uint8_t *response, uint32_t len) = 0;
  virtual bool writeQCommand(uint8_t command, uint8_t const *data, uint32_t len) = 0;
  virtual bool eraseQCommand(uint8_t command, uint32_t address) = 0;

protected:
  // Number of bytes for address
  uint8_t _addr_len;

  // Command use for read operation
  uint8_t _cmd_read;
};

#include "FlashTransport_QSPI.h"

#endif 