/**
 *  ads1260_driver.h
 *  https://www.ti.com/product/ADS1260-Q1?qgpn=ads1260-q1
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2025> (github)
 * 
 * TODO:
 * 
 */
#ifndef ADS1260_DRIVER_H
#define ADS1260_DRIVER_H

#include "ads1260_definitions.h"
// #include <Arduino.h>
#include <SPI.h>
#include "hardware_pins.h"

// #define DEBUG_ADS
#define SPI_CLK_SPEED       1000000   // Hz
#define START_UP_TIME_ADS   1150      // ms
#define INTERNAL_REF        2.498     // V
#define EXTERNAL_REF        4.997     // V - Fuente FIUNA
// #define EXTERNAL_REF        4.9362     // V - Fuente GIEM
// #define EXTERNAL_REF        5.105     // V - Cargador

extern float external_ref;

class ADS1260 {
  private:
    SPIClass* spi;
    uint8_t chipSelectPin;
    ADS1260_REGISTER_Type REGISTER;

    void sendCommand(uint8_t command);
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

  public:
    // Inicialización
    ADS1260(SPIClass* spiInterface = &SPI1, uint8_t chip_select = SPI_CS_ADC); // Por defecto SPI1
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

    // Specific metods
    float computeVolts(uint32_t value);
    float computeVolts(uint32_t value, float REFERENCE);
    void connectMUX(uint8_t p_pin, uint8_t n_pin);
    uint32_t readConversion(void);
    float readRef(void);
    float readTemperature(void);

};

#endif