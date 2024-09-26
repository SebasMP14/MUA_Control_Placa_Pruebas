/*
    Se declaran los pines útiles del MCU, teniendo de referencia el paquete
    SparkFun_SAMD51_Micromod. 
*/

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

/////// Sebas_MUA_Control_Dev
#define SCL_Sensor      PIN_WIRE_SCL        // PA16
#define SDA_Sensor      PIN_WIRE_SDA        // PA17
#define PULSE_1         PIN_I2S_SCK         // PB16
#define PULSE_2         PIN_I2S_MCK         // PB17
#define SPI_CS_MAX      PIN_SPI_SS          // PA08
#define SPI_MOSI_MAX    PIN_SPI_MOSI        // PA04
#define SPI_MISO_MAX    PIN_SPI_MISO        // PA06
#define SPI_SCK_MAX     PIN_SPI_SCK         // PA05
#define Interface_EN    D11                 // PB15

#define QSPI_D0         PIN_SPI1_SCK        // PA08     Revisar si se puede QSPI
#define QSPI_D1         PIN_SPI1_MOSI       // PA09
#define QSPI_D2         PIN_SPI1_MISO       // PA10
#define QSPI_D3         PIN_SPI1_SS         // PA11
#define QSPI_CS         D7                  // PB11     Ruteado a CS de la memoria flash
#define QSPI_SCK        D6                  // PB10     Ruteado a SCK de la memoria flash
#define UART_BOSS_rx    PIN_SERIAL1_RX      // PB30
#define UART_BOSS_tx    PIN_SERIAL1_TX      // PB31
#define UART_GPS_rx     PIN_WIRE1_SDA       // PA13
#define UART_GPS_tx     PIN_WIRE1_SCL       // PA12
#define USB_MUA_N       PIN_USB_DM          // PA24
#define USB_MUA_P       PIN_USB_PM          // PA25

/////// MICROMOD
#define PA14        D10
#define PA15        D11
#define PA18        I2CINT
#define PA19        14
#define PA20        PIN_I2S_FS
#define PA21        PIN_I2S_SDO
#define PB07        D3
#define PB08        D4
#define PB09        D5

// #define DEBUG_MAIN

#endif