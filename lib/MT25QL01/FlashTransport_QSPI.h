#ifndef FLASHTRANSPORT_QSPI_H_
#define FLASHTRANSPORT_QSPI_H_

#include "FlashTransport.h"
#include "wiring_private.h"
#include "hardware_pins.h" // AGREGADO PARA ACCEDER 
#include <Arduino.h>

class FlashTransport_QSPI : public FlashTransport {
private:
  uint8_t _sck, _cs;
  uint8_t _io0, _io1, _io2, _io3;

public:
  FlashTransport_QSPI(int8_t pinSCK, int8_t pinCS, int8_t pinIO0,
                               int8_t pinIO1, int8_t pinIO2, int8_t pinIO3);
  FlashTransport_QSPI(void);

  virtual void begin(void);
  virtual void end(void);

  virtual bool supportQuadMode(void) { return true; }

  virtual void setClockSpeed(uint32_t write_hz, uint32_t read_hz);

  virtual bool runCommand(uint8_t command);
  virtual bool readCommand(uint8_t command, uint8_t *response, uint32_t len);
  virtual bool writeCommand(uint8_t command, uint8_t const *data, uint32_t len);

  virtual bool eraseCommand(uint8_t command, uint32_t address);
  virtual bool readMemory(uint32_t addr, uint8_t *data, uint32_t len);
  virtual bool writeMemory(uint32_t addr, uint8_t const *data, uint32_t len);

  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Se agregan comandos QSPI /////////////////////
  virtual bool runQCommand(uint8_t command);
  virtual bool readQCommand(uint8_t command, uint8_t *response, uint32_t len);
  virtual bool writeQCommand(uint8_t command, uint8_t const *data, uint32_t len);
  virtual bool eraseQCommand(uint8_t command, uint32_t address);
};

#endif