/**
 *  @file hardware_pins.h
 *  
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025>
 * 
 * TODO:
 * - 
 */

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

// #define PLACA_PRUEBAS
#define PLACA_FINAL
// #define CORRECCION

/////// Sebas_MUA_Control_Dev
#ifdef PLACA_PRUEBAS

#define SCL_Sensor      PIN_WIRE_SCL        // PA16 sercom3
#define SDA_Sensor      PIN_WIRE_SDA        // PA17 sercom3
#define PULSE_1         PIN_I2S_SCK         // PB16
#define PULSE_2         PIN_I2S_MCK         // PB17
#define SPI_CS_MAX      PIN_SPI_SS          // PA07 sercom0_3
#define SPI_MOSI_MAX    PIN_SPI_MOSI        // PA04 sercom0_0
#define SPI_MISO_MAX    PIN_SPI_MISO        // PA06 sercom0_2
#define SPI_SCK_MAX     PIN_SPI_SCK         // PA05 sercom0_1
#define INTERFACE_EN    G9                 // PB15

#define QSPI_D0         PIN_QSPI_D0         // PA08     Revisar si se puede QSPI
#define QSPI_D1         PIN_QSPI_D1         // PA09
#define QSPI_D2         PIN_QSPI_D2         // PA10
#define QSPI_D3         PIN_QSPI_D3         // PA11
#define QSPI_CS         G5                  // PB11     Ruteado a CS de la memoria flash
#define QSPI_SCK        G4                  // PB10     Ruteado a SCK de la memoria flash
#define UART_BOSS_rx    PIN_SERIAL1_RX      // PB30
#define UART_BOSS_tx    PIN_SERIAL1_TX      // PB31
#define UART_GPS_rx     PIN_WIRE1_SDA       // PA13 sercom4
#define UART_GPS_tx     PIN_WIRE1_SCL       // PA12 sercom4
#define USB_MUA_N       PIN_USB_DM          // PA24
#define USB_MUA_P       PIN_USB_PM          // PA25

/////// NC ///////
// #define LED_SiPM2       PB08
// #define LED_SiPM1       PB09
// #define SPI_CS_MAX1     PIN_SPI_SS          // 
// #define SPI_CS_MAX2     G8                  // 
// #define SPI_CS_DAC1     G11                 // 
// #define SPI_CS_DAC2     G1                  // 
// #define SPI_CS_ADC      G6

/////// MICROMOD
#define PA14        38
#define PA15        39
#define PA18        I2CINT
#define PA19        14
#define PA20        PIN_I2S_FS
#define PA21        PIN_I2S_SDO
#define PB07        D3
#define PB08        D4
#define PB09        D5
#define PA01        PIN_ATN
#endif

#ifdef PLACA_FINAL

#define SCL_TMP         PIN_WIRE_SCL        // PA13 SERCOM2_1
#define SDA_TMP         PIN_WIRE_SDA        // PA12 SERCOM2_0
#define PULSE_1         G1                  // PB04
#define PULSE_2         G17                 // PB02
#define SPI_MOSI_MAX    PIN_SPI_MOSI        // PA16 SERCOM1_0
#define SPI_SCK_MAX     PIN_SPI_SCK         // PA17 SERCOM1_1
#define SPI_MISO_MAX    PIN_SPI_MISO        // PA18 SERCOM1_2
#define SPI_CS_MAX1     PIN_SPI_SS          // PA19 SERCOM1_3
#define SPI_CS_MAX2     G8                  // PA14 
#define SPI_CS_DAC1     G13                 // PA23
#define SPI_CS_DAC2     G12                 // PA22
#define INTERFACE_EN    G7                  // PB15

#define QSPI_D0         PIN_QSPI_D0         // PA08
#define QSPI_D1         PIN_QSPI_D1         // PA09
#define QSPI_D2         PIN_QSPI_D2         // PA10
#define QSPI_D3         PIN_QSPI_D3         // PA11
#define QSPI_CS         PIN_QSPI_CS         // PB11
#define QSPI_SCK        PIN_QSPI_SCK        // PB10

#ifndef  CORRECCION
#define UART_BOSS_Tx    PIN_SERIAL1_TX      // PB12 SERCOM4
#define UART_BOSS_Rx    PIN_SERIAL1_RX      // PB13 SERCOM4
#define UART_GPS_Tx     PIN_SERIAL2_TX      // PB16 SERCOM5
#define UART_GPS_Rx     PIN_SERIAL2_RX      // PB17 SERCOM5
#else
#define UART_GPS_Tx     PIN_SERIAL1_TX      // PB12 SERCOM4
#define UART_GPS_Rx     PIN_SERIAL1_RX      // PB13 SERCOM4
#define UART_BOSS_Tx    PIN_SERIAL2_TX      // PB16 SERCOM5
#define UART_BOSS_Rx    PIN_SERIAL2_RX      // PB17 SERCOM5
#endif

#define USB_MUA_N       PIN_USB_DM          // PA24
#define USB_MUA_P       PIN_USB_PM          // PA25
#define SPI_MOSI_ADC    PIN_SPI1_MOSI       // PA04 SERCOM0_0
#define SPI_SCK_ADC     PIN_SPI1_SCK        // PA05 SERCOM0_1
#define SPI_MISO_ADC    PIN_SPI1_MISO       // PA06 SERCOM0_2
#define SPI_CS_ADC      PIN_SPI1_SS         // PA07 SERCOM0_3
#define I2C_SDA1        PIN_WIRE1_SDA       // PA22 SERCOM3_0
#define I2C_SCL1        PIN_WIRE1_SCL       // PA23 SERCOM3_1

#define PA01    G0
#define PB05    G2
#define PB06    G3
#define PB07    G4
#define PB09    G5
#define PA15    G9
#define PB08    G18
#define PA20    G33
#define PA21    G34

#define LED_SiPM2   PB08
#define LED_SiPM1   PB09

#endif

#endif