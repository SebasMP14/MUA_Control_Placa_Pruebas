/**
 *  ads1260_definitions.h
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
#ifndef ADS1260_DEFINITIONS_H
#define ADS1260_DEFINITIONS_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

# ifndef __IO
#define __IO volatile
# endif

// Commands - Page 45 - Table 16
#define ADS1260_NOP             0x00
#define ADS1260_RESET           0x06
#define ADS1260_START           0x08
#define ADS1260_STOP            0x0A
#define ADS1260_RDATA           0x12
#define ADS1260_SYOCAL          0x16
#define ADS1260_GANCAL          0x17
#define ADS1260_SFOCAL          0x19
#define ADS1260_RREG            0x20
#define ADS1260_WREG            0x40
#define ADS1260_LOCK            0xF2
#define ADS1260_UNLOCK          0xF5

// Registers Map - Page 51 - Table 29
#define ADS1260_ID              0x00
#define ADS1260_STATUS          0x01
#define ADS1260_MODE0           0x02
#define ADS1260_MODE1           0x03
#define ADS1260_MODE2           0x04  // ADS1261-Q1 only
#define ADS1260_MODE3           0x05
#define ADS1260_REF             0x06
#define ADS1260_OFCAL0          0x07
#define ADS1260_OFCAL1          0x08
#define ADS1260_OFCAL2          0x09
#define ADS1260_FSCAL0          0x0A
#define ADS1260_FSCAL1          0x0B
#define ADS1260_FSCAL2          0x0C
#define ADS1260_IMUX            0x0D
#define ADS1260_IMAG            0x0E
#define ADS1260_RESERVED        0x0F
#define ADS1260_PGA             0x10
#define ADS1260_INPMUX          0x11
#define ADS1260_INPBIAS         0x12

#define ADS1260_REG_NUM         0x13 // Quantity of registers

// Values
#define ADS1260_ENABLE          0x01
#define ADS1260_DISABLE         0x00
#define ADS1260_DUMMY           0x00

// MODE0 - Page 53 - Table 32
#define ADS1260_DR_2_5          0b00000
#define ADS1260_DR_5            0b00001
#define ADS1260_DR_10           0b00010
#define ADS1260_DR_16_6         0b00011
#define ADS1260_DR_20           0b00100 // default
#define ADS1260_DR_50           0b00101
#define ADS1260_DR_60           0b00110
#define ADS1260_DR_100          0b00111
#define ADS1260_DR_400          0b01000
#define ADS1260_DR_1200         0b01001
#define ADS1260_DR_2400         0b01010
#define ADS1260_DR_4800         0b01011
#define ADS1260_DR_7200         0b01100
#define ADS1260_DR_14400        0b01101
#define ADS1260_DR_19200        0b01110
#define ADS1260_DR_25600        0b01111
#define ADS1260_DR_40000        0b10000 // etc
#define ADS1260_FILTER_SINC1    0b000
#define ADS1260_FILTER_SINC2    0b001
#define ADS1260_FILTER_SINC3    0b010
#define ADS1260_FILTER_SINC4    0b011
#define ADS1260_FILTER_FIR      0b100 // default

// MODE1 - Page 54 - Table 33
#define ADS1260_0               0b0
#define ADS1260_CHOP_NORMAL     0b00
#define ADS1260_CHOP_CHOP       0b01
#define ADS1260_CONVRT_CONT     0b0
#define ADS1260_CONVRT_PULSE    0b1
#define ADS1260_DELAY_0         0b0000 // conversion delay in milliseconds
#define ADS1260_DELAY_50000     0b0001
#define ADS1260_DELAY_59000     0b0010
#define ADS1260_DELAY_67000     0b0011
#define ADS1260_DELAY_85000     0b0100
#define ADS1260_DELAY_119000    0b0101
#define ADS1260_DELAY_189000    0b0110
#define ADS1260_DELAY_328000    0b0111
#define ADS1260_DELAY_605000    0b1000
#define ADS1260_DELAY_1_16      0b1001
#define ADS1260_DELAY_2_27      0b1010
#define ADS1260_DELAY_4_49      0b1011
#define ADS1260_DELAY_8_93      0b1100
#define ADS1260_DELAY_17_8      0b1101

// MODE2 - Page 55 - Table 34
#define ADS1260_GPIO_CON3_NOT   0b0
#define ADS1260_GPIO_CON3_YES   0b1
#define ADS1260_GPIO_CON2_NOT   0b0
#define ADS1260_GPIO_CON2_YES   0b1
#define ADS1260_GPIO_CON1_NOT   0b0
#define ADS1260_GPIO_CON1_YES   0b1
#define ADS1260_GPIO_CON0_NOT   0b0
#define ADS1260_GPIO_CON0_YES   0b1
#define ADS1260_GPIO_DIR3_OUT   0b0
#define ADS1260_GPIO_DIR3_IN    0b1
#define ADS1260_GPIO_DIR2_OUT   0b0
#define ADS1260_GPIO_DIR2_IN    0b1
#define ADS1260_GPIO_DIR1_OUT   0b0
#define ADS1260_GPIO_DIR1_IN    0b1
#define ADS1260_GPIO_DIR0_OUT   0b0
#define ADS1260_GPIO_DIR0_IN    0b1

// MODE3 - Page 56 - Table 35 (0b0 default)
#define ADS1260_PWDN_NORMAL      0b0 
#define ADS1260_PWDN_SPD         0b1
#define ADS1260_STATENB_NO      0b0 
#define ADS1260_STATENB_YES     0b1
#define ADS1260_CRCENB_NO       0b0 
#define ADS1260_CRCENB_YES      0b1
#define ADS1260_SPITIM_ARDIS    0b0
#define ADS1260_SPITIM_ARENB    0b1
#define ADS1260_GPIO_DAT3_LOW   0b0 // ADS1261-Q1 only...
#define ADS1260_GPIO_DAT3_HIGH  0b1
#define ADS1260_GPIO_DAT2_LOW   0b0
#define ADS1260_GPIO_DAT2_HIGH  0b1
#define ADS1260_GPIO_DAT1_LOW   0b0
#define ADS1260_GPIO_DAT1_HIGH  0b1
#define ADS1260_GPIO_DAT0_LOW   0b0
#define ADS1260_GPIO_DAT0_HIGH  0b1

// Status - Page 52 - Table 31
#define ADS1260_LOCK_NOT        0b0 // default
#define ADS1260_LOCK_YES        0b1
#define ADS1260_CRCERR_NO       0b0
#define ADS1260_CRCERR_YES      0b1
#define ADS1260_PGAL_ALM_NO     0b0
#define ADS1260_PGAL_ALM_YES    0b1
#define ADS1260_PGAH_ALM_NO     0b0
#define ADS1260_PGAH_ALM_YES    0b1
#define ADS1260_REFL_ALM_NO     0b0
#define ADS1260_REFL_ALM_YES    0b1
#define ADS1260_DRDY_NONEW      0b0
#define ADS1260_DRDY_NEW        0b1
#define ADS1260_CLOCK_INT       0b0
#define ADS1260_DRDY_NOINT      0b1
#define ADS1260_RESET_NO        0b0
#define ADS1260_RESET_YES       0b1 // default

// REF - Page 57 - Table 36
#define ADS1260_REFENB_INTDIS   0b0
#define ADS1260_REFENB_INTENB   0b1
#define ADS1260_RMUXP_INTPOS    0b00
#define ADS1260_RMUXP_AVDDINT   0b01    // default
#define ADS1260_RMUXP_AIN0EXT   0b10
#define ADS1260_RMUXP_AIN2EXT   0b11    // ADS1261-Q1 only
#define ADS1260_RMUXN_INTNEG    0b00
#define ADS1260_RMUXN_AVSSINT   0b01    // default
#define ADS1260_RMUXN_AIN1EXT   0b10
#define ADS1260_RMUXN_AIN3EXT   0b11    // ADS1261-Q1 only

// IMUX - Page 59 - Table 39
#define ADS1260_IMUX2_AIN0      0b0000
#define ADS1260_IMUX2_AIN1      0b0001
#define ADS1260_IMUX2_AIN2      0b0010
#define ADS1260_IMUX2_AIN3      0b0011
#define ADS1260_IMUX2_AIN4      0b0100
#define ADS1260_IMUX2_AINCOM    0b1010
#define ADS1260_IMUX2_NC        0b1111  //default
#define ADS1260_IMUX1_AIN0      0b0000
#define ADS1260_IMUX1_AIN1      0b0001
#define ADS1260_IMUX1_AIN2      0b0010
#define ADS1260_IMUX1_AIN3      0b0011
#define ADS1260_IMUX1_AINCOM    0b1010
#define ADS1260_IMUX1_AINCOM    0b1010
#define ADS1260_IMUX1_NC        0b1111  //default

// IMAG - Page 60 - Table 40
#define ADS1260_IMAG2_OFF       0b0000  // default
#define ADS1260_IMAG2_50u       0b0001
#define ADS1260_IMAG2_100u      0b0010
#define ADS1260_IMAG2_250u      0b0011
#define ADS1260_IMAG2_500u      0b0100
#define ADS1260_IMAG2_750u      0b0101
#define ADS1260_IMAG2_1000u     0b0110
#define ADS1260_IMAG2_1500u     0b0111
#define ADS1260_IMAG2_2000u     0b1000
#define ADS1260_IMAG2_2500u     0b1001
#define ADS1260_IMAG2_3000u     0b1010
#define ADS1260_IMAG1_OFF       0b0000  // default
#define ADS1260_IMAG1_50u       0b0001
#define ADS1260_IMAG1_100u      0b0010
#define ADS1260_IMAG1_250u      0b0011
#define ADS1260_IMAG1_500u      0b0100
#define ADS1260_IMAG1_750u      0b0101
#define ADS1260_IMAG1_1000u     0b0110
#define ADS1260_IMAG1_1500u     0b0111
#define ADS1260_IMAG1_2000u     0b1000
#define ADS1260_IMAG1_2500u     0b1001
#define ADS1260_IMAG1_3000u     0b1010

// PGA - Page 61 - Table 42
#define ADS1260_BYPASS_NO       0b0     // default
#define ADS1260_BYPASS_YES      0b1
#define ADS1260_GAIN_1          0b000   // default
#define ADS1260_GAIN_2          0b001 
#define ADS1260_GAIN_4          0b010 
#define ADS1260_GAIN_8          0b011 
#define ADS1260_GAIN_16         0b100 
#define ADS1260_GAIN_32         0b101 
#define ADS1260_GAIN_64         0b110
#define ADS1260_GAIN_128        0b111  

// INPMUX - Page 62 - Table 43
#define ADS1260_MUXP_AINCOM            0b0000
#define ADS1260_MUXP_AIN0              0b0001
#define ADS1260_MUXP_AIN1              0b0010
#define ADS1260_MUXP_AIN2              0b0011
#define ADS1260_MUXP_AIN3              0b0100
#define ADS1260_MUXP_AIN4              0b0101
#define ADS1260_MUXP_AIN5              0b0110 // ADS1261-Q1 only
#define ADS1260_MUXP_AIN6              0b0111 // ADS1261-Q1 only
#define ADS1260_MUXP_AIN7              0b1000 // ADS1261-Q1 only
#define ADS1260_MUXP_AIN8              0b1001 // ADS1261-Q1 only
#define ADS1260_MUXP_AIN9              0b1010 // ADS1261-Q1 only
#define ADS1260_MUXP_TEMP              0b1011
#define ADS1260_MUXP_ANALOG            0b1100
#define ADS1260_MUXP_DIGITAL           0b1101
#define ADS1260_MUXP_INOPEN            0b1110
#define ADS1260_MUXP_VCOM              0b1111
#define ADS1260_MUXN_AINCOM            0b0000
#define ADS1260_MUXN_AIN0              0b0001
#define ADS1260_MUXN_AIN1              0b0010
#define ADS1260_MUXN_AIN2              0b0011
#define ADS1260_MUXN_AIN3              0b0100
#define ADS1260_MUXN_AIN4              0b0101
#define ADS1260_MUXN_AIN5              0b0110 // ADS1261-Q1 only
#define ADS1260_MUXN_AIN6              0b0111 // ADS1261-Q1 only
#define ADS1260_MUXN_AIN7              0b1000 // ADS1261-Q1 only
#define ADS1260_MUXN_AIN8              0b1001 // ADS1261-Q1 only
#define ADS1260_MUXN_AIN9              0b1010 // ADS1261-Q1 only
#define ADS1260_MUXN_TEMP              0b1011
#define ADS1260_MUXN_ANALOG            0b1100
#define ADS1260_MUXN_DIGITAL           0b1101
#define ADS1260_MUXN_INOPEN            0b1110
#define ADS1260_MUXN_VCOM              0b1111


// REGISTER MAPPING
typedef union { // Page 51
  struct {
    uint8_t REV_ID:4;         /*!< bit:  0.. 3 Revision ID                        */
    uint8_t DEV_ID:4;         /*!< bit:  4.. 7 Device ID                          */
  } bit;                      /*!< Structure used for bit access                  */
  uint8_t reg;                /*!< Type      used for register access             */
} ADS1260_ID_Type;


typedef union { // Page 53
  struct {
    uint8_t FILTER:3;          
    uint8_t DR:5;           
  } bit;
  uint8_t reg;
} ADS1260_MODE0_Type;

typedef union { // Page 54
  struct {
    uint8_t DELAY:4;
    uint8_t CONVRT:1;          
    uint8_t CHOP:2;
    uint8_t :1;           
  } bit;
  uint8_t reg;
} ADS1260_MODE1_Type;

typedef union { // Page 55
  struct {
    uint8_t GPIO_DIR:4;
    uint8_t GPIO_CON:4;          
  } bit;
  uint8_t reg;
} ADS1260_MODE2_Type;

typedef union { // Page 56
  struct {
    uint8_t GPIO_DAT:4;
    uint8_t SPITIM:1;          
    uint8_t CRCENB:1;
    uint8_t STATENB:1;
    uint8_t PWDN:1;           
  } bit;
  uint8_t reg;
} ADS1260_MODE3_Type;

typedef union { // Page 57
  struct {
    uint8_t RMUXN:2;
    uint8_t RMUXP:2;          
    uint8_t REFENB:1;
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;           
  } bit;
  uint8_t reg;
} ADS1260_REF_Type;

typedef union { // Page 58
  struct {
    uint8_t OFC:8;            
  } bit;
  uint8_t reg;
} ADS1260_OFCAL_Type;

typedef union { // Page 58
  struct {
    uint8_t FSC:8;
  } bit;
  uint8_t reg;
} ADS1260_FSCAL_Type;

typedef union { // Page 59
  struct {
    uint8_t IMUX1:4;          
    uint8_t IMUX2:4;       
  } bit;
  uint8_t reg;
} ADS1260_IMUX_Type;

typedef union { // Page 60
  struct {
    uint8_t IMAG1:4;          
    uint8_t IMAG2:4;       
  } bit;
  uint8_t reg;
} ADS1260_IMAG_Type;

typedef union { // Page 57
  struct {
    uint8_t GAIN:3;
    uint8_t :1;          
    uint8_t :1;
    uint8_t :1;
    uint8_t :1;
    uint8_t BYPASS:1;           
  } bit;
  uint8_t reg;
} ADS1260_PGA_Type;

typedef union { // Page 62
  struct {
    uint8_t MUXN:4;           
    uint8_t MUXP:4;           
  } bit;
  uint8_t reg;
} ADS1260_INPMUX_Type;

typedef struct { // the entire register map, page 88
  __IO ADS1260_ID_Type        ID;
  // __IO ADS1260_STATUS_Type    STATUS;
  __IO ADS1260_MODE0_Type     MODE0;
  __IO ADS1260_MODE1_Type     MODE1;
  __IO ADS1260_MODE2_Type     MODE2;
  __IO ADS1260_MODE3_Type     MODE3;
  __IO ADS1260_REF_Type       REF;
  __IO ADS1260_OFCAL_Type     OFCAL0;
  __IO ADS1260_OFCAL_Type     OFCAL1;
  __IO ADS1260_OFCAL_Type     OFCAL2;
  __IO ADS1260_FSCAL_Type     FSCAL0;
  __IO ADS1260_FSCAL_Type     FSCAL1;
  __IO ADS1260_FSCAL_Type     FSCAL2;
  __IO ADS1260_IMUX_Type      IMUX;
  __IO ADS1260_IMAG_Type      IMAG;
  __IO ADS1260_PGA_Type       PGA;
  __IO ADS1260_INPMUX_Type    INPMUX;
} ADS1260_REGISTER_Type;

typedef union { // page 52
  struct {
    uint8_t RESET:1;         
    uint8_t CLOCK:1;          
    uint8_t DRDY:1;              
    uint8_t REFL_ALM:1;         
    uint8_t PGAH_ALM:1;
    uint8_t PGAL_ALM:1;
    uint8_t CRCERR:1;
    uint8_t LOCK:1;               
  } bit;
  uint8_t reg;
} ADS1260_STATUS_Type;

#ifdef __cplusplus
}
#endif

#endif