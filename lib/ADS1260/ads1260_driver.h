#ifndef ADS1260_DRIVER_H
#define ADS1260_DRIVER_H

#include "ads1260_definitions.h"
// #include <Arduino.h>
#include <SPI.h>
#include "hardware_pins.h"

#define DEBUG_ADS
#define SPI_CLK_SPEED 1000000

class ADS1260 {
  private:
    SPIClass* spi;
    uint8_t chipSelectPin;

    void sendCommand(uint8_t command);
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

  public:
    // Inicialización
    ADS1260(SPIClass* spiInterface = &SPI1, uint8_t chip_select = SPI_CS_ADC);
    void begin(void);
    void setStartPin(uint8_t pin); // designate a pin connected to START

    // Control Commands
    void noOperation(void);
    void reset(void);
    void start(void);
    void stop(void);

    // Read Data Command
    uint32_t readData(uint8_t p_pin, uint8_t n_pin);

    // Calibration Commands
    void sysOffsetCalibration(uint8_t shorted1, uint8_t shorted2);
    void gainCalibration(uint8_t vcc_pin, uint8_t vss_pin);
    void selfOffsetCalibration(void);

    // Register Commands
    uint32_t readRegisterData(uint8_t registro);
    void writeRegisterData(uint8_t registro, uint8_t data);

    // Protection Commands
    void registerLock(void);
    void registerUnlock(void);

};

#endif