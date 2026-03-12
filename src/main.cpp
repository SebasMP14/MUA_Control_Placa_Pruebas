/**
 * @file main.cpp
 * Control de la misión M.U.A. (Monitoring Unit of the South Anomaly)
 *
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 *
 * Made by:
 * - Sebas Monje <2024-2025> (github) → amonje@fiuna.edu.py
 *
 * TODO:
 *
 */

#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
#include "mcp4561_driver.h"
#include "max1932_driver.h"
#include "dac8551_driver.h"
#include "ads1260_driver.h"
#include "calculos.h"
#include "obc_comm.h"
#include "power_manager.h"

// #define DEBUG_MAIN
// #define DEBUG_
// #define PRUEBA_CATODO
// #define FLUX
// #define WITHOUT_DETECTION_BOARD
// #define DESACTIVE_CHANNEL_1
// #define DESACTIVE_CHANNEL_2
// #define FIRST_DETECTION_BOARD
#define SECOND_DETECTION_BOARD              // Placa con rangos modificados
// #define THIRD_DETECTION_BOARD
// #define DEBUG_NEW_POL_SETTLING
// #define PLACA_CONTROL_V3

/**
 * FIXED_BIAS_MODE: Prueba de flujo con voltaje de polarización fijo.
 *
 * Casos de prueba:
 *   Caso 1 → FIXED_BIAS_MODE activo, temperatura ambiente  (~25 °C)
 *   Caso 2 → FIXED_BIAS_MODE activo, temperatura elevada   (~42 °C)
 *   Caso 3 → Comentar FIXED_BIAS_MODE, compensación activa, temperatura elevada (~42 °C)
 *            → se espera flujo del Caso 3 ≈ flujo del Caso 1
 *
 * Para Casos 1 y 2: dejar activo el define de abajo.
 * Para Caso 3:      comentarlo →  // #define FIXED_BIAS_MODE
 */
// #define FIXED_BIAS_MODE

#define MAX_ITER 5                     // Protocol initialization attempts
#define Ventana 5                      // Para Sliding Moving Average
#define Switching_Time_MAX 4           // Microseconds
#define TRAMA_DATA_SIZE 36             //
#define TRAMA_INFO_SIZE 8              // 39 Bytes maximum
#define TRAMA_CURVE_SIZE 39            // VER LA FORMA DE DISMINUIR LA CANT DE PUNTOS (400)
#define RESISTIVE_DIVISOR 11.79272944f // Average value

#ifndef DESACTIVE_CHANNEL_1
// Parametros de canal 1
uint8_t pot1 = 0xD9;              // 0xE8; Placa de deteccion V2// 0xDD Prueba final con tutores 14/06
uint8_t MAX_INIT_1 = 0x01;        // 0x44; --- //0x60 --- // 0x01;
float MIN_VOUT_SIPM_1 = 25.1705f; // 27.92f(single)/22.039(ambos-on) --- // 26.714(single)/20.7283(ambos-on) --- //24.595(single)/25.1705(ambos-on)
const float ov1 = 2.67f;          // Establecido por el fabricante: 2.5 V
#endif

#ifndef DESACTIVE_CHANNEL_2
// Parametros de canal 2
uint8_t pot2 = 0xD9;              // 0xE8; Placa de deteccion V2// 0xDD Prueba final con tutores 14/06
uint8_t MAX_INIT_2 = 0x01;        // 0x44; --- //0x60 --- // 0x01;
float MIN_VOUT_SIPM_2 = 25.0917f; // 21.96486(single)/21.9622(ambos-on) ---  //20.659(single)/20.659(ambos-on) ---  // 25.104(single)/25.0917(ambos-on)
const float ov2 = 3.0f;           // Establecido por el fabricante: 2.5 V
#endif

uint8_t status = 0;
uint8_t state = 0x01;
// uint32_t timestamp = 0;
uint8_t segundos = 120; // Calibración cada tantos segundos
const float Voffset = 3.829428571f;
const float voffset = Voffset / RESISTIVE_DIVISOR;
const float ResisA = 1050;

#ifdef FIRST_DETECTION_BOARD
const float ResisB = 2000;
const float ResisC = 12700;
// const float OverVoltage = 0.2238233f * ov // CALCULAR
const float OverVoltage = ov / RESISTIVE_DIVISOR;
uint8_t MAX_INIT = 0x01;      // 0x44     // 0x60;
float MIN_VOUT_SIPM = 24.595; // 27.92f;  // 26.714f;  // Teórico
#endif
#ifdef SECOND_DETECTION_BOARD
const float ResisB = 1000;
const float ResisC = 10000;
#ifndef DESACTIVE_CHANNEL_1
float OverVoltage1 = ov1 / RESISTIVE_DIVISOR;
#endif
#ifndef DESACTIVE_CHANNEL_2
float OverVoltage2 = ov2 / RESISTIVE_DIVISOR;
#endif
#endif
#ifdef THIRD_DETECTION_BOARD
const float ResisB = 1000;
const float ResisC = 12400;
#ifndef DESACTIVE_CHANNEL_1
float OverVoltage1 = ov1 / RESISTIVE_DIVISOR;
#endif
#ifndef DESACTIVE_CHANNEL_2
float OverVoltage2 = ov2 / RESISTIVE_DIVISOR;
#endif
#endif
uint16_t DAC_INIT = 0x7FFF;

float firstCurrent1 = 0.0f;
float firstCurrent2 = 0.0f;
float temperature1 = -273.0f;
float temperature2 = -273.0f;
float temperature = -273.0f;
float searchMargin = 1.5f; // Search Vbd window
const uint16_t Elementos1 = 400;
const uint16_t Elementos2 = 400;
union FloatToUint32
{ // Para evitar aliasing y no violar las reglas del compilador
  float f;
  uint32_t u;
};
FloatToUint32 lati;
FloatToUint32 longi;
FloatToUint32 temp;
FloatToUint32 vbd1;
FloatToUint32 vbd2;
FloatToUint32 vcurr1;
FloatToUint32 vcurr2;
unsigned long time_ini = 0x00;
unsigned long time_flag = 0x00;
unsigned long time_flow = 0x00; // Para flujo de cuentas cada 10 segundos
unsigned long timestamp = 0x00;
uint16_t last_count1 = 0;
uint16_t last_count2 = 0;
uint32_t total_count1 = 0;
uint32_t total_count2 = 0;
float Lat = 0.0f;
float Long = 0.0f;
float Vbd1 = 0.0f;   // Breakdown Voltage Channel one
float Vbias1 = 0.0f; // Polarization Voltage Channel one
float Vcurr1 = 0.0f; // Breakdown Current Voltage Channel one
float Vbd2 = 0.0f;   // Breakdown Voltage Channel two
float Vbias2 = 0.0f; // Polarization Voltage Channel two
float Vcurr2 = 0.0f; // Breakdown Current Voltage Channel two

bool flag1 = false; // Para imprimir las lecturas de ADC de cada canal
bool flag2 = false;

ADS1260 ads1260;

uint8_t sendTrama[TRAMA_DATA_SIZE] = {0x26};

// Almacenan los datos
uint16_t indexPeak1 = 0;
uint16_t indexPeak2 = 0;
uint16_t inverseVoltage_command1[Elementos1]; // Comandos a ser enviados a cada DAC
uint16_t inverseVoltage_command2[Elementos2]; // Comandos a ser enviados a cada DAC

// Channel 1
float inverseVoltage1[Elementos1];  // Voltaje en el SiPM
float inverseVCurrent1[Elementos1]; // Corriente
float Filtered_voltage1[Elementos1];
float Filtered_current1[Elementos1];
float temperatureArray1[Elementos1];
// Channel 2
float inverseVoltage2[Elementos2];  // Voltaje en el SiPM
float inverseVCurrent2[Elementos2]; // Corriente
float Filtered_voltage2[Elementos2];
float Filtered_current2[Elementos2];
float temperatureArray2[Elementos2];

void setupCOUNT(void);
void loopCOUNT(void);
void setupTRANSFER(void);
void loopTRANSFER(void);
void loopTRANSFERinfo(void);

void obtain_Curve_inverseVI(float Temperature, uint8_t CS_DAC, float REFERENCE);
float polarization_settling(float Vbd, uint8_t CS_DAC);
bool sendDataFrame(void);
bool sendInfoFrame(void);
void printArrays_ch1(void);
void printArrays_ch2(void);

bool enable_Interface(void);

void setup()
{
  delay(4000);

  Serial.begin(115200); // Puerto USB
#ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial Iniciado");
#endif

#ifdef DEBUG_MAIN
// Serial.println("PRUEBA DE FLUJO DE PARTICULAS 22/05/2025, SOLO CHANNEL 1, Without Radiactive Coin");
#endif

  Serial1.begin(9600); // OBC (On Board Computer)
#ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial1 Iniciado");
#endif

  /* Inicialización de memoria Flash */
  if (!start_flash())
  { // Se utiliza en ambos modos de operación
// break;
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (setup) -> Flash con problemss");
#endif
  }

  // uint8_t escribir[36] = { 0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
  //                         0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
  //                         0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
  //                         0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01};
  // write_mem(escribir, 36);
  // uint16_t len = 200;
  // uint8_t leer[len] = {};
  // read_until(leer, len);
  // for (uint8_t i = 0; i < len; i++) {
  //   Serial.print(" 0x");
  //   Serial.print(leer[i], HEX);
  // }
  // Serial.println();

  // while (true) {}

  // if ( erase_debug() ) {
  //   #ifdef DEBUG_MAIN
  //   Serial.println("DEBUG (setup) -> debug borrado");
  //   #endif
  // } else {
  //   #ifdef DEBUG_MAIN
  //   Serial.println("DEBUG (setup) -> No se pudo borrar debug");
  //   #endif
  // }

  // if ( erase_all() ) {
  //   Serial.println("Flash borrada");
  // }

  // Restaurar último estado guardado en memoria
  write_OPstate(0x00);
  delay(1000);
  get_OPstate(&state);
  // state = 0x14; // For COUNT test only

  pinMode(PULSE_1, INPUT_PULLDOWN);
  pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(PA01, OUTPUT); // Salida para TC2 (utiliza también PA15)
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_SiPM1, OUTPUT);
  pinMode(LED_SiPM2, OUTPUT);
  pinMode(INTERFACE_EN, OUTPUT);
  digitalWrite(INTERFACE_EN, LOW);

#ifdef DEBUG_MAIN
  Serial.print("Estado: 0x");
  Serial.println(state, HEX);
#endif

  switch (state)
  {          // STATE obtained from the flash memory
  case 0x00: // STAND_BY
  case 0xFF:
    currentMode = STAND_BY;
    requestOperationMode(); // Espera del modo de operación
    if (currentMode == COUNT_MODE)
    {
      setupCOUNT();
    }
    else if (currentMode == TRANSFER_DATA_MODE)
    {
      setupTRANSFER();
    }
    else if (currentMode == TRANSFER_INFO_MODE)
    {
      setupTRANSFER();
    }
    break;

  case 0x01: // COUNT_MODE
    currentMode = COUNT_MODE;
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (setup) -> COUNT_MODE iniciado");
#endif
    setupCOUNT();
    break;

  case 0x02: // TRANSFER_DATA_MODE
    currentMode = TRANSFER_DATA_MODE;
    setupTRANSFER();
    break;

    // case 0x08:                               // Eliminar por que podría entrar en un bucle...
    //   currentMode = FINISH;
    //   enterOffMode();
    //   break;

  case 0x09: // TRANSFER_DATA_MODE
    currentMode = TRANSFER_INFO_MODE;
    setupTRANSFER();
    break;

  default:
    /* Modo no seleccionado o incorrecto, manejar... */
    currentMode = STAND_BY;
    break;
  }

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Setup finalizado...");
#endif
}

void loop()
{
  switch (currentMode)
  {
  case STAND_BY:
    requestOperationMode();
    if (currentMode == COUNT_MODE)
    {
      setupCOUNT();
    }
    else if (currentMode == TRANSFER_DATA_MODE)
    {
      setupTRANSFER();
    }
    else if (currentMode == TRANSFER_INFO_MODE)
    {
      setupTRANSFER();
    }
    break;

  case COUNT_MODE:
    loopCOUNT();
    if (Serial1.available())
    {
      requestOperationMode();
      if (currentMode == TRANSFER_DATA_MODE)
      {
        setupTRANSFER();
      }
      else if (currentMode == TRANSFER_INFO_MODE)
      {
        setupTRANSFER();
      }
    }
    break;

  case TRANSFER_DATA_MODE:
    loopTRANSFER(); // La verificación de un nuevo comando se hace en la función
    break;

  case TRANSFER_INFO_MODE:
    loopTRANSFERinfo(); // La verificación de un nuevo comando se hace en la función
    break;

  case FINISH:
    write_OPstate(0x00);
    enterOffMode();
    break;

  default:
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (loop) -> UNKNOWN_MODE");
#endif
    delay(2000);
    requestOperationMode();
    // if ( !setup_state && currentMode != UNKNOWN_MODE ) {
    //   setupCOUNT();
    // }
    /* Modo no seleccionado o incorrecto, manejar... */
    break;
  }
}

/******************************************************
 * @fn      setupCOUNT - loopCOUNT
 * @brief   - Setup: Inicializaciones del modo de operación: Conteo y Procesamiento
 *          - Loop: Cada vez que detect_TC sea true se procesan los datos, mientras se cuentan los pulsos.
 * @param   NONE
 * @return  NONE
 * TODO: - Se debe establecer la tasa de cambio de la temperatura para ajustar el tiempo de
 *        interrupcion del timer counter
 */
void setupCOUNT(void)
{
  SPI.begin();  // BUS MAX1y2 & DAC1y2
  Wire.begin(); // BUS TMP100 y MCP4561
  Wire.setClock(400000);

  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (rtc.begin())
    { // Configuración del RTC
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de RTC exitosa.");
#endif
      break;
    }
#ifdef DEBUG_MAIN
    Serial.print("DEBUG (setupCOUNT) -> Inicialización de RTC fallida: ");
    Serial.println(iter_counter);
#endif
    delay(10);
  }
  // getTimestampFromGPS();                            // Falta Programar...

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_SiPM1, LOW);
  digitalWrite(LED_SiPM2, LOW);
  digitalWrite(INTERFACE_EN, HIGH); // Activación de Placa Interfaz(5V-3V3) y ADC1260(5V)

#ifndef WITHOUT_DETECTION_BOARD
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP0(0x9C))
    { // Configuración del MCP4561
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP0(), HEX);
#endif

#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP1(0x9C))
    { // Configuración del MCP4561
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP1(), HEX);
#endif
#endif

  delay(START_UP_TIME_ADS); // Habilitación del ADC (REVISAR TIEMPO)

#ifndef WITHOUT_DETECTION_BOARD
  start_dac8551(SPI_CS_DAC1); // SiPM 1
  start_max1932(SPI_CS_MAX1);

  start_dac8551(SPI_CS_DAC2); // SiPM 2
  start_max1932(SPI_CS_MAX2);

  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (start_tmp100())
    { // Configuración del TMP100
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#endif

  // ADC Configuration
  ads1260.begin();
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN); // respuesta esperada: 00100100
#endif
  ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100); // 40 KSPS - FIR (Page 30)
  // ads1260.writeRegisterData(ADS1260_MODE0, 0b01101100);           // 14400 SPS
  delay(50); // single_shoot
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);
  Serial.print("DEBUG (setupCOUNT) -> PGA: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
#endif
  ads1260.writeRegisterData(ADS1260_PGA, 0b10000000); // BYPASS MODE
  ads1260.writeRegisterData(ADS1260_REF, 0b00001001);
  delay(50);
// ads1260.readRegister
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> PGA BYPASS MODE: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
#endif
  // ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000);           // STATENB  REVISARRRRRRRRRRRRRRRRRRRRRRRR
  // ads1260.writeRegisterData(ADS1260_REF, 0b00010000);             // REF 2.498V ENABLE
  delay(300);

  // external_ref = ads1260.readRef();                             // Se lee la referencia
  external_ref = 3.923650f;
  // ads1260.writeRegisterData(ADS1260_REF, 0b00001001);

  // #ifdef DEBUG_MAIN
  // Serial.print("DEBUG (loopCOUNT) -> readRef: ");
  // Serial.println(external_ref, 6);
  // #endif
  // Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM), external_ref), 6);

  // while(true){
  //   delay(1000);
  // }

#ifdef DEBUG_
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // 0x7FFF
  write_max_reg(MAX_INIT, SPI_CS_MAX1);     // 0x40
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC2); // 0x7FFF
  write_max_reg(MAX_INIT, SPI_CS_MAX2);     // 0x40
  delay(100);
  float val1, val2, val3, val4;
  while (true)
  {
    val1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN1, ADS1260_MUXN_AINCOM), external_ref);
    delay(100);
    val2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
    delay(100);
    val3 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref);
    delay(100);
    val4 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN4, ADS1260_MUXN_AINCOM), external_ref);
    Serial.println(val1, 6);
    Serial.println(val2, 6);
    delay(500);
    Serial.println(val3, 6);
    Serial.println(val4, 6);
    delay(10000);
    Serial.println("val1, val3");
    Serial.print((val1 * RESISTIVE_DIVISOR) - Voffset, 7);
    Serial.print(", ");
    Serial.print((val3 * RESISTIVE_DIVISOR) - Voffset, 7);
  }
#endif

// #ifndef WITHOUT_DETECTION_BOARD
// Primera polarización de los SiPMs
// Channel 1
#ifdef PRUEBA_CATODO
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // Activación de Vout1 al mínimo valor
  write_max_reg(MAX_INIT_1, SPI_CS_MAX1);
  delayMicroseconds(300);
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC2); // Activación de Vout2 al mínimo valor
  write_max_reg(MAX_INIT_2, SPI_CS_MAX2);
  while (true)
  {
  }
#endif

#ifndef DESACTIVE_CHANNEL_1
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // Activación de Vout1 al mínimo valor
  write_max_reg(MAX_INIT_1, SPI_CS_MAX1);
  delayMicroseconds(300);

  temperature1 = read_tmp100();
#ifdef DEBUG_MAIN
  Serial.print("Temperatura1: ");
  Serial.println(temperature1, 4);
#endif
#ifdef PLACA_CONTROL_V3
  firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#else
  firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref);
#endif
#ifdef DEBUG_MAIN
  Serial.print("firstCurrent1: ");
  Serial.println(firstCurrent1, 6);
#endif
  obtain_Curve_inverseVI(temperature1, SPI_CS_DAC1, external_ref);
  sliding_moving_average(inverseVoltage1, Elementos1, Ventana, Filtered_voltage1);
  sliding_moving_average(inverseVCurrent1, Elementos1, Ventana, Filtered_current1);
  Vbd1 = obtain_Vbd(Filtered_current1, Filtered_voltage1, Elementos1, &Vcurr1, &indexPeak1);
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (obtain_Vbd) -> Command DAC del Vbd1 obtenido: 0x");
  Serial.println(inverseVoltage_command1[indexPeak1], HEX);
  Serial.print("DEBUG (obtain_Vbd) → Vi_Vbd = ");
  Serial.println(Vcurr1, 6);
  Serial.print("DEBUG (obtain_Vbd) → Isipm = ");
  Serial.println(SiPMCurrent(Vcurr1, firstCurrent1), 8);
#endif
// write_dac8551_reg(inverseVoltage_command1[indexPeak1], SPI_CS_DAC1);
// Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref), 6);
// while (true) {}
// external_ref = ads1260.readRef();                             // Se lee la referencia
#ifdef DEBUG_MAIN
  // Serial.print("DEBUG (loopCOUNT) -> readRef: ");
  // Serial.println(external_ref, 6);
  Serial.print("Over Voltage: ");
  Serial.println(ov1, 2);
#endif

  // write_dac8551_reg(DAC_INIT, SPI_CS_DAC1);                       // Activación de Vout1 al mínimo valor
  // write_max_reg(MAX_INIT, SPI_CS_MAX1);
  // delayMicroseconds(300);
  Vbias1 = polarization_settling(Vbd1, SPI_CS_DAC1);

#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP0(pot1))
    { // Configuración del MCP4561
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP0(), HEX);
#endif
  delay(100);
  // activeInterrupt1();                                           // Una vez polarizado
  flag1 = true;
#endif

#ifndef DESACTIVE_CHANNEL_2
// Channel 2
// external_ref = ads1260.readRef();                             // Se lee la referencia
#ifdef DEBUG_MAIN
// Serial.print("DEBUG (loopCOUNT) -> readRef: ");
// Serial.println(external_ref, 6);
#endif
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC2); // Activación de Vout2 al mínimo valor
  write_max_reg(MAX_INIT_2, SPI_CS_MAX2);

  delayMicroseconds(300);
  temperature2 = read_tmp100();
#ifdef DEBUG_MAIN
  Serial.print("Temperatura2: ");
  Serial.println(temperature2, 4);
#endif
#ifdef PLACA_CONTROL_V3
  firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN4, ADS1260_MUXN_AINCOM), external_ref);
#else
  firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#endif
#ifdef DEBUG_MAIN
  Serial.print("firstCurrent2: ");
  Serial.println(firstCurrent2, 6);
#endif
  obtain_Curve_inverseVI(temperature2, SPI_CS_DAC2, external_ref);
  sliding_moving_average(inverseVoltage2, Elementos2, Ventana, Filtered_voltage2);
  sliding_moving_average(inverseVCurrent2, Elementos2, Ventana, Filtered_current2);
  Vbd2 = obtain_Vbd(Filtered_current2, Filtered_voltage2, Elementos2, &Vcurr2, &indexPeak2);
  Vbias2 = polarization_settling(Vbd2, SPI_CS_DAC2);
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP1(pot2))
    { // Configuración del MCP4561
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP1(), HEX);
#endif

  // activeInterrupt2();                                     // Una vez polarizado
  flag2 = true;
#endif
  // #endif

  noInterrupts();
  pulse_count1 = 0;
  pulse_count2 = 0;
  interrupts();
  delay(500);
  activeInterrupt1();
  activeInterrupt2();

  setup_state = true;
#ifdef DEBUG_MAIN
  Serial.println("setupCount finalizado...");
#endif
  time_ini = millis();
  time_flag = millis();
  time_flow = millis();
  last_count1 = 0;
  last_count2 = 0;

  setupTC2(segundos);
}

void loopCOUNT(void)
{
  if (detect1)
  { // Se puede obtener el ancho del pulso?, es necesario?: (si, no)
    detect1 = false;
#ifdef DEBUG_MAIN
    Serial.print("COUNT1: ");
    Serial.println(pulse_count1);
#endif
  }

  if (detect2)
  {
    detect2 = false;
#ifdef DEBUG_MAIN 
    Serial.print("COUNT2: ");
    Serial.println(pulse_count2);
#endif
  }

  // if ( (millis() - time_flag) >= 480000 ) {
  //   flag1 = true;
  //   flag2 = true;
  //   time_flag = millis();
  // }

  if ((millis() - time_ini) >= 20000)
  { /////////// Rutina de prueba
    // read_all();
    // digitalWrite(LED_BUILTIN, HIGH);                      // Blink
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    //-----------------------------------------------------------------------
    /*Serial.print("millis: ");
    time_ini = millis();
    Serial.print(time_ini);
    Serial.print(", timestamp: ");
    Serial.print(rtc.now().unixtime());
    #ifndef WITHOUT_DETECTION_BOARD
    Serial.print(", Temperatura: ");
    Serial.println(read_tmp100(), 4);
    //-----------------------------------------------------------------------
    // Serial.print("Vv_bias, Vi_bias: ");
    // Serial.print(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM), external_ref), 6);
    // Serial.print(", ");
    // Serial.print(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref), 6);
    // Serial.print(", ");
    // Serial.print(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN1, ADS1260_MUXN_AINCOM), external_ref), 6);
    // Serial.print(", ");
    // Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref), 6);
    #endif*/

    // for (uint8_t i = 0xFF; i > 0x00; i -= 0x10) {
    //   delay(10000);
    //   #ifdef DEBUG_MAIN
    //   Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
    //   #endif
    //   for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    //     if ( writeMCP0(i) ) {                         // Configuración del MCP4561
    //       #ifdef DEBUG_MAIN
    //       Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
    //       #endif
    //       break;
    //     } else {
    //       #ifdef DEBUG_MAIN
    //       Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
    //       Serial.println(iter_counter);
    //       #endif
    //       delay(10);
    //     }
    //   }
    //   #ifdef DEBUG_MAIN
    //   Serial.print("MCP escrito en: ");
    //   Serial.println(readMCP0(), HEX);
    //   #endif
    // }
  }

/**   Interrupción del TC2 cada 60 seg: Primeramente se deben desactivar las interrupciones de los pulsos,
 * luego se debe guardar en memoria ->
 * timestamp(4B) - LatyLong(2x4B) - temperature(4B) - Count1y2(2x2B) - Vbias1y2(2x4B) - Vcurr1y2(2x4B) → 36 Bytes,
 * realizar el algoritmo de polarización y obtener Vbias1y2, inicializar las variables globales y activar
 * las interrupciones de los pulsos.
 *    El proceso de calibración sucede un canal a la vez. */
// Flujo de cuentas cada 10 segundos
#ifdef FLUX
  if ((millis() - time_flow) >= 1000 && !detect_TC)
  {
    unsigned long elapsed = millis() - time_flow;
    time_flow = millis();

    noInterrupts();
    uint16_t snap1 = pulse_count1;
    uint16_t snap2 = pulse_count2;
    interrupts();

    uint16_t delta1 = snap1 - last_count1;
    uint16_t delta2 = snap2 - last_count2;
    last_count1 = snap1;
    last_count2 = snap2;

    total_count1 += delta1; // ↓ AGREGÁ ESTO ↓
    total_count2 += delta2;

    float temp_now = read_tmp100();
#ifdef PLACA_CONTROL_V3
    float vbias1_now = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN1, ADS1260_MUXN_AINCOM), external_ref) * RESISTIVE_DIVISOR;
    float vbias2_now = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref) * RESISTIVE_DIVISOR;
#endif

    Serial.print("FLUJO -> CH1: ");
    Serial.print(delta1);
    Serial.print(" cnt|CH2: ");
    Serial.print(delta2);
    Serial.print(" cnt|Vbias1: ");
    Serial.print(vbias1_now - Voffset - ov1, 4);
    Serial.print(" V|Vbias2: ");
    Serial.print(vbias2_now - Voffset - ov2, 4);
    Serial.print(" V|T: ");
    Serial.print(temp_now, 4);
    Serial.print(" C");

    Serial.print(" | Total CH1: ");
    Serial.print(total_count1);
    Serial.print(" | Total CH2: ");
    Serial.println(total_count2);
  }
#endif

  if (detect_TC)
  {
    detect_TC = false;
    disableTC2();
    desactiveInterrupt1();
    desactiveInterrupt2();
    // noInterrupts();

#ifndef DESACTIVE_CHANNEL_2
    // temperature = (temperature1 + temperature2) / 2;    // Average between curve measurements
    temperature = temperature2;
#else
    temperature = temperature1;
#endif

    timestamp = getTime();
    lati.f = Lat;
    longi.f = Long;
    temp.f = temperature;
    vbd1.f = Vbd1;
    vbd2.f = Vbd2;
    vcurr1.f = Vcurr1;
    vcurr2.f = Vcurr2;

    if (isnan(temperature))
    {
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Error al leer la temperatura.");
#endif
      // manejar - Se puede leer el del ADC
    }

// read lat y long from GPS
// timestamp - Lat - Long - temperature - Count1y2 - Vbd1y2 - Vcurr1y2
#ifdef DEBUG_MAIN
    Serial.print("TRAMA -> timestamp: (");
    Serial.print(timestamp);
    Serial.print(", 0x");
    Serial.print(timestamp, HEX);
    Serial.print("), Lat: (");
    Serial.print(lati.f);
    Serial.print(", 0x");
    Serial.print(lati.u, HEX);
    Serial.print("), Long: (");
    Serial.print(longi.f);
    Serial.print(", 0x");
    Serial.print(longi.u, HEX);
    Serial.print("), TMP: (");
    Serial.print(temp.f, 4);
    Serial.print(" ºC, 0x");
    Serial.print(temp.u, HEX);
    Serial.print("), Count1: (");
    Serial.print(pulse_count1);
    Serial.print(", 0x");
    Serial.print(pulse_count1, HEX);
    Serial.print("), Count2: (");
    Serial.print(pulse_count2);
    Serial.print(", 0x");
    Serial.print(pulse_count2, HEX);
    Serial.print("), Vbd1: (");
    Serial.print(vbd1.f);
    Serial.print(", 0x");
    Serial.print(vbd1.u, HEX);
    Serial.print("), Vbd2: (");
    Serial.print(vbd2.f);
    Serial.print(", 0x");
    Serial.print(vbd2.u, HEX);
    Serial.print("), Vcurr1: (");
    Serial.print(vcurr1.f);
    Serial.print(", 0x");
    Serial.print(vcurr1.u, HEX);
    Serial.print("), Vcurr2: (");
    Serial.print(vcurr2.f);
    Serial.print(", 0x");
    Serial.print(vcurr2.u, HEX);
    Serial.println(")");
#endif
// pulse_count
#ifdef DEBUG__
    read_all();
#endif // descomentar

/* GUARDADO Little-Endian */
// if ( !write_mem((uint8_t *)&timestamp, sizeof(timestamp)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura timestamp.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&lati.u, sizeof(lati.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Lat.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&longi.u, sizeof(longi.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Long.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&temp.u, sizeof(temperature)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura temperatura.");
//   #endif
// }
// desactiveInterrupt1();
// if ( !write_mem((uint8_t *)&pulse_count1, sizeof(pulse_count1)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura pulse_count1.");
//   #endif
// }
// // pulse_count1 = 0;
// desactiveInterrupt2();
// if ( !write_mem((uint8_t *)&pulse_count2, sizeof(pulse_count2)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura pulse_count2.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&vbd1.u, sizeof(vbd1.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Vbd1.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&vbd2.u, sizeof(vbd2.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Vbd2.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&vcurr1.u, sizeof(vcurr1.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Vcurr1.");
//   #endif
// }
// if ( !write_mem((uint8_t *)&vcurr2.u, sizeof(vcurr2.u)) ) {
//   #ifdef DEBUG_MAIN
//   Serial.println("ERROR (loopCOUNT) -> Fallo en la escritura Vcurr2.");
//   #endif
// }

// enable_Interface();                                           // LINEA DE PRUEBA

// external_ref = ads1260.readRef();                             // Se lee nuevamente la ref del ADC
// #ifdef DEBUG_MAIN
// Serial.print("DEBUG (loopCOUNT) -> readRef: ");
// Serial.println(external_ref, 6);
// #endif
// desactiveInterrupt1();
// #ifndef WITHOUT_DETECTION_BOARD
// Channel 1 Polarization
#ifndef DESACTIVE_CHANNEL_1
#ifdef FIXED_BIAS_MODE
    // MODO VOLTAJE FIJO: se omite la curva IV y el reajuste del DAC.
    // El voltaje aplicado en setupCOUNT permanece intacto.
    // Solo se lee temperatura para el log de FLUJO.
    temperature1 = read_tmp100();
#else
    // MODO COMPENSACIÓN: recalcula Vbd con la temperatura actual y reajusta Vbias.
    write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // Vout1 al mínimo valor
    write_max_reg(MAX_INIT_1, SPI_CS_MAX1);
    delayMicroseconds(300);
    temperature1 = read_tmp100();
// #ifdef DEBUG_MAIN
// Serial.print("Temperatura1: ");
// Serial.println(temperature1, 4);
// #endif
#ifdef PLACA_CONTROL_V3
    firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#else
    firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref);
#endif
    // #ifdef DEBUG_MAIN
    // Serial.println("VVoltage, VCorriente");
    // Serial.print(firstVoltage1, 6);
    // Serial.print(", ");
    // Serial.println(firstCurrent1, 6);
    // #endif
    obtain_Curve_inverseVI(temperature1, SPI_CS_DAC1, external_ref);
    sliding_moving_average(inverseVoltage1, Elementos1, Ventana, Filtered_voltage1);           // Voltage Filtering
    sliding_moving_average(inverseVCurrent1, Elementos1, Ventana, Filtered_current1);          // Current Filtering
    Vbd1 = obtain_Vbd(Filtered_current1, Filtered_voltage1, Elementos1, &Vcurr1, &indexPeak1); //
    // #ifdef DEBUG_MAIN
    // Serial.print("DEBUG (obtain_Vbd) -> command del Vbd1 obtenido: ");
    // Serial.println(inverseVoltage_command1[indexPeak1], HEX);
    // #endif
    Vbias1 = polarization_settling(Vbd1, SPI_CS_DAC1);
#endif // FIXED_BIAS_MODE
// Vbias1=24.9478f;
#ifdef DEBUG_MAIN
    Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
    for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
    {
      if (writeMCP0(pot1))
      { // Configuración del MCP4561
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
        break;
      }
      else
      {
#ifdef DEBUG_MAIN
        Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
        Serial.println(iter_counter);
#endif
        delay(10);
      }
    }
#ifdef DEBUG_MAIN
    Serial.print("MCP escrito en: ");
    Serial.println(readMCP0(), HEX);
#endif
    delay(100);
    // activeInterrupt1();
    flag1 = true;
#endif

#ifndef DESACTIVE_CHANNEL_2
#ifdef FIXED_BIAS_MODE
    // MODO VOLTAJE FIJO: se omite la curva IV y el reajuste del DAC.
    // El voltaje aplicado en setupCOUNT permanece intacto.
    // Solo se lee temperatura para el log de FLUJO.
    temperature2 = read_tmp100();
#else
    // MODO COMPENSACIÓN: recalcula Vbd con la temperatura actual y reajusta Vbias.
    write_dac8551_reg(DAC_INIT, SPI_CS_DAC2); // Activación de Vout2 al mínimo valor
    write_max_reg(MAX_INIT_2, SPI_CS_MAX2);
    delayMicroseconds(300);
    temperature2 = read_tmp100();
#ifdef DEBUG_MAIN
    Serial.print("Temperatura2: ");
    Serial.println(temperature2, 4);
#endif
#ifdef PLACA_CONTROL_V3
    firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN4, ADS1260_MUXN_AINCOM), external_ref);
#else
    firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#endif
#ifdef DEBUG_MAIN
    Serial.print("firstCurrent2: ");
    Serial.println(firstCurrent2, 6);
#endif
    obtain_Curve_inverseVI(temperature2, SPI_CS_DAC2, external_ref);
    sliding_moving_average(inverseVoltage2, Elementos2, Ventana, Filtered_voltage2);
    sliding_moving_average(inverseVCurrent2, Elementos2, Ventana, Filtered_current2);
    Vbd2 = obtain_Vbd(Filtered_current2, Filtered_voltage2, Elementos2, &Vcurr2, &indexPeak2);
    Vbias2 = polarization_settling(Vbd2, SPI_CS_DAC2);
#endif // FIXED_BIAS_MODE
// Vbias2 = 24.9422;
#ifdef DEBUG_MAIN
    Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
    for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
    {
      if (writeMCP1(pot2))
      { // Configuración del MCP4561
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
        break;
      }
      else
      {
#ifdef DEBUG_MAIN
        Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
        Serial.println(iter_counter);
#endif
        delay(10);
      }
    }
#ifdef DEBUG_MAIN
    Serial.print("MCP escrito en: ");
    Serial.println(readMCP1(), HEX);
#endif
    // activeInterrupt2();                                     // Una vez polarizado
    flag2 = true;
#endif
    // #endif

    // noInterrupts();
    // pulse_count1 = 0;
    // pulse_count2 = 0;
    // interrupts();
    delay(100); // PRUEBA 05/03/2026
    activeInterrupt1();
    activeInterrupt2();
    enableTC2();
  }

  /* Para imprimir los puntos de la curva */
  if (flag1 && !detect_TC)
  {
    flag1 = false;
    printArrays_ch1();
  }
  if (flag2 && !detect_TC)
  {
    flag2 = false;
    printArrays_ch2();
  }
}

/**********************************************************************************
 * @fn      setupTRANSFER - loopTRANSFER
 * @brief   - Setup: Inicializaciones del modo de operación: Transferencia de Datos
 *          - Loop: Se transfieren datos y se revisa comandos entrantes
 * @param   NONE
 * @return  NONE
 * @todo    - test
 */
void setupTRANSFER(void)
{
#ifdef DEBUG_MAIN
  Serial.println("DEBUG (setupTRANSFER) -> Iniciado ");
#endif

  digitalWrite(INTERFACE_EN, LOW); // Desactive detection board
  disableTC2();                    // Disable time interrupt
  desactiveInterrupt1();           // Disable external interrupts
  desactiveInterrupt2();

#ifdef DEBUG_MAIN
  read_all(); // To compare the data
  uint32_t start_address = 0x00000948;
  write_SENT_DATAaddress(&start_address); // To transfer from the begining of the flash
#endif

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (setupTRANSFER) -> setupTRANSFER finalizado...");
#endif
  setup_state = true;
}

void loopTRANSFER(void)
{
  uint8_t buffer[TRAMA_COMM] = {0};
  /* AGREGAR ACK */ // Hacer tambien lo de enviar los paquetes de datos disponibles antes de enviar los datos cuando
                    // entra en modo transferencia...
  if (slidingWindowBuffer(buffer, timeOUT_window))
  { // Se busca y revisa una trama válida proveniente del OBC
    if (verifyOBCResponse(buffer))
    { // NACK se maneja en la función
      ack_MUA_to_OBC[1] = buffer[1];
      Serial1.write(ack_MUA_to_OBC, TRAMA_COMM); // SEND ACKNOWLEDGE FRAME
      switch (buffer[1])
      {
      case ID_STANDBY:
        currentMode = STAND_BY;
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (requestOperationMode) -> STAND_BY ACTIVATED");
#endif
        write_OPstate(ID_STANDBY);
        return;
        break;
      case ID_COUNT_MODE:
        currentMode = COUNT_MODE;
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (loopTRANSFER) -> COUNT MODE ACTIVATED");
#endif
        write_OPstate(ID_COUNT_MODE);
        enable_Interface();
        return;
        break;
      case ID_TRANSFER_MODE:
        currentMode = TRANSFER_DATA_MODE;
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (loopTRANSFER) -> TRANSFER MODE ACTIVATED");
#endif
        write_OPstate(ID_TRANSFER_MODE);
        return;
        break;
      case ID_FINISH:
        currentMode = FINISH;
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (loopTRANSFER) -> FINISH MODE ACTIVATED");
        Serial.println("Sleep mode in progress: Executing order 66.");
#endif
        write_OPstate(ID_STANDBY);
        return;
        break;
      case ID_TRANSFER_SYSINFO_MODE:
        currentMode = TRANSFER_INFO_MODE;
#ifdef DEBUG_MAIN
        Serial.println("DEBUG (loopTRANSFER) -> TRANSFER SYSINFO MODE ACTIVATED");
#endif
        write_OPstate(ID_TRANSFER_SYSINFO_MODE);
        return;
        break;
      default:
        /* ADD INVALID FRAME */
        // currentMode = STAND_BY;
        // #ifdef DEBUG_MAIN
        // Serial.println("DEBUG (requestOperationMode) -> UNKNOWN MODE");
        // #endif
        // write_OPstate(ID_STANDBY);
        // return ;
        break;
      } // switch (buffer[1])
    } // verifyOBCResponse
  } // slidingWindowBuffer

  if (!sendDataFrame())
  { // Durante sendDataFrame se pueden recibir comandos del OBC, manejar
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (loopTRANSFER) -> Falló el envío de trama."); // Then, we try again the same frame
#endif
    return;
  }
}

void loopTRANSFERinfo(void)
{
  if (!sendInfoFrame())
  {
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (loopTRANSFERinfo) -> Falló el envío de trama."); // Then, we try again the same frame
#endif
    return;
  }
}

/************************************************************************************************************
 * @fn      obtain_Curve_inverseVI
 * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del ADC.
 * @param   Temperature Obtenido del sensor TMP100 para la estimación teórica
 * @param   CS_DAC Channel selection
 * @return  NONE
 */
void obtain_Curve_inverseVI(float Temperature, uint8_t CS_DAC, float REFERENCE)
{
  uint8_t muxP0, muxP1, led;
  uint16_t Elementos;
  uint16_t *inverseVoltage_command;
  float MIN_VOUT_SIPM; // Mínimo valor de tensión a suministrar al SiPM para obtener la curva inversa
  float *inverseVoltage;
  float *inverseVCurrent;
  // float* temperatureArray;
  

#ifdef PLACA_CONTROL_V3
#ifndef DESACTIVE_CHANNEL_1
  muxP0 = ADS1260_MUXP_AIN1; // Initialization
  muxP1 = ADS1260_MUXP_AIN3;
  MIN_VOUT_SIPM = MIN_VOUT_SIPM_1;
  Elementos = Elementos1;
  led = LED_SiPM1;
  inverseVoltage = inverseVoltage1;
  inverseVCurrent = inverseVCurrent1;
  inverseVoltage_command = inverseVoltage_command1;
// temperatureArray  = temperatureArray1;
#endif
  if (CS_DAC == SPI_CS_DAC2)
  {
#ifndef DESACTIVE_CHANNEL_2
    muxP0 = ADS1260_MUXP_AIN2;
    muxP1 = ADS1260_MUXP_AIN4;
    MIN_VOUT_SIPM = MIN_VOUT_SIPM_2;
    Elementos = Elementos2;
    led = LED_SiPM2;
    inverseVoltage = inverseVoltage2;
    inverseVCurrent = inverseVCurrent2;
    inverseVoltage_command = inverseVoltage_command2;
// temperatureArray  = temperatureArray2;
#endif
  }
#else
  #ifndef DESACTIVE_CHANNEL_1
  muxP0 = ADS1260_MUXP_AIN0; // Initialization
  muxP1 = ADS1260_MUXP_AIN2;
  MIN_VOUT_SIPM = MIN_VOUT_SIPM_1;
  Elementos = Elementos1;
  led = LED_SiPM1;
  inverseVoltage = inverseVoltage1;
  inverseVCurrent = inverseVCurrent1;
  inverseVoltage_command = inverseVoltage_command1;
// temperatureArray  = temperatureArray1;
  #endif
  if (CS_DAC == SPI_CS_DAC2)
  {
    #ifndef DESACTIVE_CHANNEL_2
    muxP0 = ADS1260_MUXP_AIN1;
    muxP1 = ADS1260_MUXP_AIN3;
    MIN_VOUT_SIPM = MIN_VOUT_SIPM_2;
    Elementos = Elementos2;
    led = LED_SiPM2;
    inverseVoltage = inverseVoltage2;
    inverseVCurrent = inverseVCurrent2;
    inverseVoltage_command = inverseVoltage_command2;
// temperatureArray  = temperatureArray2;
    #endif
  }
#endif

  float Vbd_Teo = Vbd_teorical(Temperature);
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (obtain_Curve_inverseVI) -> Vbd teorico: ");
  Serial.println(Vbd_Teo, 6);
#endif
  float MAX_VOUT_SIPM = MIN_VOUT_SIPM + 12;
  float Vlimite_inferior = max(MIN_VOUT_SIPM, Vbd_Teo + Voffset - searchMargin); // 24.588 es el minimo valor a la salida del MAX
  float Vlimite_superior = min(MAX_VOUT_SIPM, Vbd_Teo + Voffset + searchMargin); // 36 es el máximo valor a la salida del MAX
  uint16_t Vlim_inf = VDAC_command(MAX_VOUT_SIPM, Vlimite_inferior);             // Comando       // revisar el valor de la formula
  uint16_t Vlim_sup = VDAC_command(MAX_VOUT_SIPM, Vlimite_superior);             // Comando
  uint16_t paso = (uint16_t)(Vlim_inf - Vlim_sup) / Elementos;
  unsigned long start_time, total_time;

#ifdef DEBUG_MAIN

  Serial.print("DEBUG (obtain_Curve_inverseVI) -> Limites (dec, hex): ");
  Serial.print(out_voltage(MAX_VOUT_SIPM, Vlim_inf), 4);
  Serial.print(", ");
  Serial.print(out_voltage(MAX_VOUT_SIPM, Vlim_sup), 4);
  Serial.print(", 0x");
  Serial.print(Vlim_inf, HEX);
  Serial.print(", 0x");
  Serial.print(Vlim_sup, HEX);
  Serial.print(", paso: ");
  Serial.println(paso);

#endif
  digitalWrite(led, HIGH);

  start_time = micros();
  for (uint16_t i = 0; i < Elementos; i++)
  {
    inverseVoltage_command[i] = Vlim_inf - i * paso; // Comandos para el DAC
    write_dac8551_reg(inverseVoltage_command[i], CS_DAC);
    delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    delay(5);                              // Settling time of the MAX (Vout)
    // Considerar el tiempo de asentamiento del filtro pasa bajos de 2ndo orden...

    inverseVoltage[i] = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), REFERENCE);
    inverseVCurrent[i] = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), REFERENCE);
    // temperatureArray1[i] = read_tmp100();
  }

  total_time = micros() - start_time;
  // Serial.println("ya");
  // Serial.println(inverseVoltage1[399], 6);
  // Serial.println(inverseVCurrent1[399], 6);
  // while(true){}
  write_dac8551_reg(DAC_INIT, CS_DAC); // Mínimo valor de tensión suministrado
  digitalWrite(led, LOW);              // Se debe apagar el LED
  float Ts = (float)total_time / (Elementos * 1000);
#ifdef DEBUG_MAIN
  Serial.print("Tiempo promedio de muestreo [ms]: ");
  Serial.println(Ts, 4);
#endif
}

/************************************************************************************************************
 * @fn      polarization_settling
 * @brief   Establecimiento del voltaje de polarización de un canal
 * @param   Vbd: Breakdown Voltage
 * @param   CS_DAC: Channel to polarizate
 * @return  float Vbias (Polarization Voltage)
 */
float polarization_settling(float Vbd, uint8_t CS_DAC)
{
  float Vbias = 0.0f;
  uint8_t muxP0, muxP1, led;
  float firstCurrent, ov;
  uint16_t indexPeak = 0, *inverseVoltage_command;

#ifdef PLACA_CONTROL_V3
#ifndef DESACTIVE_CHANNEL_1
  muxP0 = ADS1260_MUXP_AIN1; // Configuración de lectura: Canal 1 o 2
  muxP1 = ADS1260_MUXP_AIN3;
  led = LED_SiPM1;
  inverseVoltage_command = inverseVoltage_command1;
  firstCurrent = firstCurrent1;
  indexPeak = indexPeak1;
  ov = ov1;
#endif

  if (CS_DAC == SPI_CS_DAC2)
  {
#ifndef DESACTIVE_CHANNEL_2
    muxP0 = ADS1260_MUXP_AIN2;
    muxP1 = ADS1260_MUXP_AIN4;
    led = LED_SiPM2;
    inverseVoltage_command = inverseVoltage_command2;
    firstCurrent = firstCurrent2;
    indexPeak = indexPeak2;
    ov = ov2;
#endif
    Serial.println("DEBUG (polarization_settling) → Channel 2");
  }
  else
  {
    Serial.println("DEBUG (polarization_settling) → Channel 1");
  }
#else
  #ifndef DESACTIVE_CHANNEL_1
  muxP0 = ADS1260_MUXP_AIN0; // Configuración de lectura: Canal 1 o 2
  muxP1 = ADS1260_MUXP_AIN2;
  led = LED_SiPM1;
  inverseVoltage_command = inverseVoltage_command1;
  firstCurrent = firstCurrent1;
  indexPeak = indexPeak1;
  ov = ov1;
  #endif
  if (CS_DAC == SPI_CS_DAC2)
  {
    #ifndef DESACTIVE_CHANNEL_2
    muxP0 = ADS1260_MUXP_AIN1;
    muxP1 = ADS1260_MUXP_AIN3;
    led = LED_SiPM2;
    inverseVoltage_command = inverseVoltage_command2;
    firstCurrent = firstCurrent2;
    indexPeak = indexPeak2;
    ov = ov2;
#endif
    Serial.println("DEBUG (polarization_settling) → Channel 2");
  }
#endif

#ifdef DEBUG_MAIN
// Serial.println("DEBUG (polarization_settling) → Channel 1");
// Serial.print("DEBUG (polarization_settling) → iniciando \nDEBUG (polarization_settling) → firstCurrent = ");
// Serial.println(firstCurrent, 6);
#endif

#ifndef DEBUG_NEW_POL_SETTLING

  uint16_t i = 0x0020;
  uint16_t command = inverseVoltage_command[indexPeak];
  // uint16_t command = DAC_INIT;
  Vbias = (Vbd * RESISTIVE_DIVISOR) + ov;
  float Vv = 0;
  digitalWrite(led, HIGH);
  while (Vv * RESISTIVE_DIVISOR <= Vbias)
  {
    command -= i;
    write_dac8551_reg(command, CS_DAC); // Disminuir el comando -> aumento de Vout

    // delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    // delay(5); // Settling time of the MAX (Vout)
    // Considerar el tiempo de asentamiento del filtro pasa bajos de 2ndo orden... // 354.96 useg

    Vv = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), external_ref);
    // Serial.print("Vv, Vi: ");
    // Serial.print(Vv, 7);
    // Serial.print(", ");
    // Serial.println(ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref), 7);

    if (command < 0x0040)
    { // evitamos underflow
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (polarization_settling) -> Limit reached");
#endif
      break;
    }
  }

#ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) -> Vbias_command: ");
  Serial.println(command, HEX);
  Serial.print("DEBUG (polarization_settling) -> Vbias: ");
  Serial.println(Vv, 6);
  Serial.print("DEBUG (polarization_settling) -> VIbias: ");
  Serial.println(ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref), 6);
#endif
  digitalWrite(led, LOW);

  return Vv;
#else
  Vbias = (Vbd * RESISTIVE_DIVISOR) + ov; // en el Vbd ya se encuentran los 3.8V de offset
  uint16_t Vbias_DAC_CMD = CMD_DAC(MAX_INIT, Vbias) - 0x000A;
// external_ref = ads1260.readRef();
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) → external_ref = ");
  Serial.println(external_ref, 6);
#endif
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) → Iniciando en Vbias_calculado, DAC_CMD_Vbias, Vbias_readed, Vbias_: ");
  Serial.print(Vbias, 4);
  Serial.print(", 0x");
  Serial.print(Vbias_DAC_CMD, HEX);
  Serial.print(", ");
#endif
  write_dac8551_reg(Vbias_DAC_CMD, CS_DAC);
  delay(3);
  uint8_t i = 0x30;
  float Vi1, Vi2, Vi3, Vtia1, Vtia2, Vtia3, Vv;
  uint16_t aux, aux_;

  digitalWrite(led, HIGH);
  Vv = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), external_ref);
  delay(3);
#ifdef DEBUG_MAIN
  Serial.print(Vv, 6);
  Serial.print(", ");
  Serial.println(Vv * RESISTIVE_DIVISOR, 6);
#endif
  Vi1 = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref);
  Vtia1 = Vtia(Vi1, firstCurrent);
  Vbias_DAC_CMD -= i;
  write_dac8551_reg(Vbias_DAC_CMD, CS_DAC);
  delay(3);
  Vi2 = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref);
  Vtia2 = Vtia(Vi2, firstCurrent);
  Vbias_DAC_CMD -= i;
  write_dac8551_reg(Vbias_DAC_CMD, CS_DAC);
  delay(3);
  Vi3 = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref);
  Vtia3 = Vtia(Vi3, firstCurrent);

  while (Vtia1 - Vtia2 < Vtia2 - Vtia3 || Vv * RESISTIVE_DIVISOR <= Vbias)
  { // Vbias calculado == al Vbias leido
    aux = Vi2;
    aux_ = Vtia2;
    Vi1 = aux;
    Vtia1 = aux_;
    Vi2 = Vi3;
    Vtia2 = Vtia3;
    Vbias_DAC_CMD -= i;
    write_dac8551_reg(Vbias_DAC_CMD, CS_DAC);
    delay(3);
    Vi3 = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref);
    Vtia3 = Vtia(Vi3, firstCurrent);

#ifdef DEBUG_MAIN
    Serial.print("DEBUG (polarization_settling) → Vtia2: ");
    Serial.print(Vtia2, 6);
    Serial.print(", 0x");
    Serial.print(Vbias_DAC_CMD, HEX);
    Serial.print(", Calculated Current = ");
    Serial.println(SiPMCurrent(Vi2, firstCurrent), 8);
#endif

    Vv = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), external_ref);
    if (Vv * RESISTIVE_DIVISOR >= Vbias)
    {
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (polarization_settling) → Vbias_calculated equal readed");
#endif
      break;
    }
  }

  Vbias_DAC_CMD += i;
  write_dac8551_reg(Vbias_DAC_CMD, CS_DAC);
  delay(3);
  Vi2 = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), external_ref);
  Vtia2 = Vtia(Vi2, firstCurrent);
  float vbias = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), external_ref);
  digitalWrite(led, LOW);

  uint8_t CMD_POT = VMCP_to_DEC(Vtia2);
  // CMD_POT = 0xF0;

#ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) → writing in MCP: ");
  Serial.println(CMD_POT, HEX);
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP0(CMD_POT))
    {
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (polarization_settling) → CMD sent to MCP4561 successful.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (polarization_settling) → CMD sent to MCP4561 failed: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) → MCP escrito en: ");
  Serial.println(readMCP0(), HEX);
  Serial.print("DEBUG (polarization_settling) → Vbias_Objective, vbias, Vbias_setted: ");
  Serial.print(Vbias, 6);
  Serial.print(", ");
  Serial.print(vbias, 6);
  Serial.print(", ");
  Serial.println(vbias * RESISTIVE_DIVISOR, 6);
#endif

  return Vbias;
#endif
}

/************************************************************************************************************
 * @fn      sendDataFrame
 * @brief   Envía una trama de datos al OBC, recibe ACK o NACK y ejecuta en consecuencia
 * @param   void
 * @return  true: Transmisión exitosa, ACK recibido ...
 * @return  false: Transmisión fallida, CRC invalide, data frame invalid or timeout
 * @todo    - Al esperar ACK solo debe ir timeOUT_invalid_frame
 */
bool sendDataFrame(void)
{
  uint32_t last_address_written = 0xFFFFFFFF;
  uint32_t last_sent_address = 0xFFFFFFFF;

  if (!get_address(&last_address_written))
    return false;
  if (!get_SENT_DATAaddress(&last_sent_address))
    return false;

  if (last_sent_address == 0xFFFFFFFF)
    last_sent_address = 0x00; // primer envío de día uno (*festeja*)
  if (last_address_written == last_sent_address)
  { // ya no quedan datos en memoria por enviar
#ifdef DEBUG_MAIN
    Serial.println("DEBUG (sendDataFrame) → last_address_written == last_sent_address");
#endif
    currentMode = STAND_BY;
    write_OPstate(STAND_BY);
    return true; // All data available in flash memory sent
  }

  uint8_t trama_size = TRAMA_DATA_SIZE + TRAMA_COMM;
  uint8_t trama[trama_size] = {0};
  if (!buildDataFrame(trama, ID_SENT_DATA, TRAMA_DATA_SIZE, last_sent_address))
    return false;

  Serial1.write(trama, trama_size);

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (sendDataFrame) -> Trama enviada:");
  for (uint8_t i = 0; i < trama_size; i++)
  {
    Serial.print(" 0x");
    Serial.print(trama[i], HEX);
  }
  Serial.println();
#endif

  // unsigned long tiempo = millis();
  // while ( Serial1.available() < TRAMA_COMM ) { // usar la funcionn, esta ya tiene un timeout
  //   if ( (millis() - tiempo) >= timeOUT ) return false;
  // }
  uint8_t recibido[TRAMA_COMM];
  // Serial1.readBytes(recibido, TRAMA_COMM);
  if (!slidingWindowBuffer(recibido, timeOUT))
  { // Solo debe ir timeOUT_invalid_frame
// delay(timeOUT_invalid_frame);                   // Se tiene que eliminar
// Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);   // Se pidio deshabilitar, reu 10/05/2025
#ifdef DEBUG_MAIN
    Serial.println("ERROR (sendDataFrame) → Fallo slidingWindowBuffer");
#endif
    return false;
  }

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (sendDataFrame) -> Respuesta recibida:");
  for (uint8_t i = 0; i < TRAMA_COMM; i++)
  {
    Serial.print(" 0x");
    Serial.print(recibido[i], HEX);
  }
  Serial.println();
#endif

  // sacar la verificacion del CRC por que este es constante (0xAAAA)
  // if ( !verifyOBCResponse(recibido) ) return false;   // REVISAR, se maneja el invalid frame también
  if (!verifyCRCACK(recibido))
    return false; // El CRC es siempre 0xAAAA para el ACK

  if (recibido[1] == trama[1])
  { // ACK CMD_ID is the same as data sent
    last_sent_address += TRAMA_DATA_SIZE;
    write_SENT_DATAaddress(&last_sent_address); // se actualiza la siguiente dirección a enviar
  }
  else if (recibido[1] == ID_FINISH)
  {
    currentMode = FINISH;
  }
  else if (recibido[1] == ID_TRANSFER_SYSINFO_MODE)
  {
    currentMode = TRANSFER_INFO_MODE;
  }
  else if (recibido[1] == ID_COUNT_MODE)
  {
    currentMode = COUNT_MODE;
  }
  else if (recibido[1] == ID_STANDBY)
  {
    currentMode = STAND_BY;
  }

#ifdef DEBUG_MAIN
  digitalWrite(LED_BUILTIN, HIGH); // Blink
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  return true;
}

bool sendInfoFrame(void)
{
  uint8_t trama_size = TRAMA_COMM + TRAMA_INFO_SIZE;
  uint8_t trama[trama_size];

  if (!buildDataFrame(trama, ID_TRANSFER_SYSINFO_MODE, TRAMA_INFO_SIZE, SAVED_ADDRESS_SECTOR_DIR))
    return false;

  Serial1.write(trama, trama_size);

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (sendInfoFrame) -> Trama enviada:");
  for (uint8_t i = 0; i < trama_size; i++)
  {
    Serial.print(" 0x");
    Serial.print(trama[i], HEX);
  }
  Serial.println();
#endif

  uint8_t recibido[TRAMA_COMM];
  if (!slidingWindowBuffer(recibido, timeOUT))
  {
#ifdef DEBUG_MAIN
    Serial.println("ERROR (sendInfoFrame) → Fallo slidingWindowBuffer");
#endif
    return false;
  }

#ifdef DEBUG_MAIN
  Serial.println("DEBUG (sendInfoFrame) -> Respuesta recibida:");
  for (uint8_t i = 0; i < TRAMA_COMM; i++)
  {
    Serial.print(" 0x");
    Serial.print(recibido[i], HEX);
  }
  Serial.println();
#endif

  if (!verifyCRCACK(recibido) && recibido[1] != trama[1])
    return false;

  currentMode = STAND_BY; // or STAND_BY

  return true;
}

bool enable_Interface(void)
{
  digitalWrite(LED_SiPM1, LOW);
  digitalWrite(LED_SiPM2, LOW);
  digitalWrite(INTERFACE_EN, HIGH); // Activación de Placa Interfaz(5V-3V3) y ADC1260(5V)

#ifndef WITHOUT_DETECTION_BOARD
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
#endif
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (writeMCP0(0x9C))
    { // Configuración del MCP4561
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#ifdef DEBUG_MAIN
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP0(), HEX);
#endif
#endif

  delay(START_UP_TIME_ADS); // Habilitación del ADC (REVISAR TIEMPO)

#ifndef WITHOUT_DETECTION_BOARD
  for (uint8_t iter_counter = 0; iter_counter <= MAX_ITER; iter_counter++)
  {
    if (start_tmp100())
    { // Configuración del TMP100
#ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 exitosa.");
#endif
      break;
    }
    else
    {
#ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida: ");
      Serial.println(iter_counter);
#endif
      delay(10);
    }
  }
#endif

  // ADC Configuration
  ads1260.begin();
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN); // respuesta esperada: 00100100
#endif
  ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100); // 40 KSPS - FIR (Page 30)
  // ads1260.writeRegisterData(ADS1260_MODE0, 0b01101100);           // 14400 SPS
  delay(50);
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);
  Serial.print("DEBUG (setupCOUNT) -> PGA: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
#endif
  ads1260.writeRegisterData(ADS1260_PGA, 0b10000000); // BYPASS MODE
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> PGA BYPASS MODE: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
#endif
  // ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000);           // STATENB  REVISARRRRRRRRRRRRRRRRRRRRRRRR
  // ads1260.writeRegisterData(ADS1260_REF, 0b00010000);             // REF 2.498V ENABLE
  delay(300);

// external_ref = ads1260.readRef();                             // Se lee la referencia
#ifdef DEBUG_MAIN
// Serial.print("DEBUG (loopCOUNT) -> readRef: ");
// Serial.println(external_ref, 6);
#endif

#ifdef DEBUG_
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // 0x7FFF
  write_max_reg(MAX_INIT, SPI_CS_MAX1);     // 0x40
  uint16_t aux = DAC_INIT;
  while (true)
  {
// aux -= 0x0100;
// external_ref = ads1260.readRef();                             // Se lee la referencia
#ifdef DEBUG_MAIN
// Serial.print("DEBUG (loopCOUNT) -> readRef: ");
// Serial.println(external_ref, 6);
#endif
    write_dac8551_reg(0x4235, SPI_CS_DAC1); /* Medi 28.67 V con el multimetro (channel 1)*/
    delay(5);                               /* Debieron ser 34V */
    Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM), external_ref), 6);
    Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref), 6);
    delay(500);
    Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM), external_ref), 6);
    Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref), 6);
    delay(10000);
  }
#endif

#ifndef WITHOUT_DETECTION_BOARD
// Primera polarización de los SiPMs
#ifndef DESACTIVE_CHANNEL_1
  // Channel 1
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC1); // Activación de Vout1 al mínimo valor
  write_max_reg(MAX_INIT_1, SPI_CS_MAX1);
  temperature1 = read_tmp100();
#ifdef DEBUG_MAIN
  Serial.print("Temperatura: ");
  Serial.println(temperature1, 4);
#endif
#ifdef PLACA_CONTROL_V3
  firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#else
  firstCurrent1 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref);
#endif
#ifdef DEBUG_MAIN
  Serial.print("firstCurrent1: ");
  Serial.println(firstCurrent1, 6);
#endif
  obtain_Curve_inverseVI(temperature1, SPI_CS_DAC1, external_ref);
  sliding_moving_average(inverseVoltage1, Elementos1, Ventana, Filtered_voltage1);
  sliding_moving_average(inverseVCurrent1, Elementos1, Ventana, Filtered_current1);
  Vbd1 = obtain_Vbd(Filtered_current1, Filtered_voltage1, Elementos1, &Vcurr1, &indexPeak1);
#ifdef DEBUG_MAIN
  Serial.print("DEBUG (obtain_Vbd) -> command del Vbd obtenido: ");
  Serial.println(inverseVoltage_command1[indexPeak1], HEX);
#endif
  // write_dac8551_reg(inverseVoltage_command[indexPeak1], SPI_CS_DAC1);
  // Serial.println(ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM), external_ref), 6);
  // while (true) {}
  Vbias1 = polarization_settling(Vbd1, SPI_CS_DAC1);
  // Vbias1=24.9412f;
  activeInterrupt1(); // Una vez polarizado
  flag1 = true;
#ifdef DEBUG_MAIN
  Serial.print("Over Voltage: ");
  Serial.println(ov1, 2);
#endif
#endif

#ifndef DESACTIVE_CHANNEL_2
// Channel 2
// external_ref = ads1260.readRef();                             // Se lee la referencia
#ifdef DEBUG_MAIN
// Serial.print("DEBUG (loopCOUNT) -> readRef: ");
// Serial.println(external_ref, 6);
#endif
  write_dac8551_reg(DAC_INIT, SPI_CS_DAC2); // Activación de Vout2 al mínimo valor
  write_max_reg(MAX_INIT_2, SPI_CS_MAX2);
  temperature2 = read_tmp100();
#ifdef DEBUG_MAIN
  Serial.print("Temperatura2: ");
  Serial.println(temperature2, 4);
#endif
#ifdef PLACA_CONTROL_V3
  firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN4, ADS1260_MUXN_AINCOM), external_ref);
#else
  firstCurrent2 = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM), external_ref);
#endif
  obtain_Curve_inverseVI(temperature2, SPI_CS_DAC2, external_ref);
  sliding_moving_average(inverseVoltage2, Elementos2, Ventana, Filtered_voltage2);
  sliding_moving_average(inverseVCurrent2, Elementos2, Ventana, Filtered_current2);
  Vbd2 = obtain_Vbd(Filtered_current2, Filtered_voltage2, Elementos2, &Vcurr2, &indexPeak2);
  Vbias2 = polarization_settling(Vbd2, SPI_CS_DAC2);
  activeInterrupt2(); // Una vez polarizado
  flag1 = true;
#endif
#endif

  setupTC2(segundos);

  setup_state = true;
#ifdef DEBUG_MAIN
  Serial.println("setupCount finalizado...");
#endif
  time_ini = millis();

  return true;
}

void printArrays_ch1(void)
{
  Serial.println("-------------------- DATOS OBTENIDOS --------------------");
  Serial.println("------------------ -------------------");
  Serial.println("i, Voltage, VCorriente, T, (V*12)-3.8-Ra*(VCorr-firstCurr1)/Rb, (VCorr-firstCurr1)/Rb");
  for (uint16_t i = 1; i < Elementos1; i++)
  {
    Serial.print(i);
    Serial.print(",");
    Serial.print(inverseVoltage1[i], 7);
    Serial.print(",");
    Serial.print(inverseVCurrent1[i], 7);
    Serial.print(",");
    Serial.print(temperatureArray1[i]);
    Serial.print(",");
    Serial.print((inverseVoltage1[i] * RESISTIVE_DIVISOR) - Voffset - (ResisA * (inverseVCurrent1[i] - firstCurrent1) / ResisB), 7);
    Serial.print(",");
    Serial.println((inverseVCurrent1[i] - firstCurrent1) / ResisB, 10);
  }
}

void printArrays_ch2(void)
{
  Serial.println("-------------------- DATOS OBTENIDOS --------------------");
  Serial.println("------------------ -------------------");
  Serial.println("i, Voltage, VCorriente, T, (V*12)-3.8-Ra*(VCorr-firstCurr1)/Rb, (VCorr-firstCurr1)/Rb");
  for (uint16_t i = 1; i < Elementos2; i++)
  {
    Serial.print(i);
    Serial.print(",");
    Serial.print(inverseVoltage2[i], 7);
    Serial.print(",");
    Serial.print(inverseVCurrent2[i], 7);
    Serial.print(",");
    Serial.print(temperatureArray2[i]);
    Serial.print(",");
    Serial.print((inverseVoltage2[i] * RESISTIVE_DIVISOR) - Voffset - (ResisA * (inverseVCurrent2[i] - firstCurrent2) / ResisB), 7);
    Serial.print(",");
    Serial.println((inverseVCurrent2[i] - firstCurrent2) / ResisB, 10);
  }
}

/*
  Codigo de prueba

 #ifdef DEBUG__
  uint8_t counter = 0;
  uint8_t X = 3;
//////////////////////// Enviar
  uint8_t trama_size = 30 + TRAMA_COMM;
  uint8_t trama[trama_size] = {MISSION_ID, ID_SENT_DATA, 0x1E, 0x4A, 0x47, 0x36, 0x59, 0x42,
                              0x57, 0x30, 0x4A, 0x47, 0x36, 0x59, 0x50, 0x59, 0x30, 0x3E, 0xF0, 0xAA, 0x03,
                              0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78};

  uint16_t CRC = crc_calculate(trama);
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (loopTRANSFER) -> CRC calculado: ");
  Serial.println(CRC, HEX);
  #endif
  trama[trama_size-3] = (uint8_t)(CRC >> 8);
  trama[trama_size-2] = (uint8_t)(CRC & 0xFF);
  trama[trama_size-1] = 0x0A;               // STOP BYTE


  while ( Serial1.available() ) {
    Serial1.read();
  }
  Serial1.write(trama, trama_size);         // Envío de trama
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (loopTRANSFER) -> Datos enviados: ");
  for ( uint8_t i = 0; i < trama_size; i++ ) {
    Serial.print(" 0x");
    Serial.print(trama[i], HEX);
  }
  Serial.println();
  #endif

//////////////////////////// Recibir
  while ( Serial1.available() < TRAMA_COMM );           // esperando ACK de OBC

  uint8_t recibido[TRAMA_COMM];
  Serial1.readBytes(recibido, TRAMA_COMM);
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (loopTRANSFER) -> respuesta: ");
  for ( uint8_t i = 0; i < TRAMA_COMM; i++ ) {
    Serial.print(" 0x");
    Serial.print(recibido[i], HEX);
  }
  Serial.println();
  #endif

  CRC = crc_calculate(recibido);       // TRAMA_COMM o index_begin
  uint16_t crc_received = (recibido[TRAMA_COMM-3] << 8) | recibido[TRAMA_COMM-2];

  if ( CRC != crc_received ) {
    while ( Serial1.available() ) { // CUIDADO - REVISAR flush serial1
      Serial1.read();
    }
    Serial1.write(nack_MUA_to_OBC, TRAMA_COMM);     // invalid checksum NACK
    Serial.println("Invalid checksum NACK");
    // return ;
  } else if ( recibido[0] != MISSION_ID ) {
    delay(timeOUT_invalid_frame);
    while ( Serial1.available() ) {// CUIDADO - REVISAR flush serial1
      Serial1.read();
    }
    Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);  //  invalid frame NACK
    Serial.println("Invalid frame NACK enviado");
    // return ;
  }

  if ( recibido[1] != ACK_OBC_to_MUA ) {
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loopTRANSFER) -> recibido[1] != ACK_OBC_to_MUA");
    #endif
  }

  if (counter == X) {
    currentMode = FINISH;
  }
  #endif
*/

/*
// transferir datos
  // read_all();
  delay(1000);
  *
  * Esta sección prueba el envío de los datos al OBC, los únicos datos enviados son los que contiene "trama"
  * y se envía X veces



  #ifdef DEBUG_
  unsigned long tiempo = 0;
  uint32_t last_address_written = 0xFFFFFFFF;
  get_address(&last_address_written);

  uint8_t trama_size = TRAMA_DATA_SIZE + TRAMA_COMM;
  uint8_t trama[trama_size] = {MISSION_ID, ID_TRANSFER_MODE, TRAMA_DATA_SIZE};
  uint32_t last_sent_address = 0xFFFFFFFF;

  ****************************************************************************************************
  *                                       Starting Sending Data
  ***************************************************************************************************
  if ( get_SENT_DATAaddress(&last_sent_address) && last_sent_address == 0xFFFFFFFF ) { // Primera vez en enviar datos
    last_sent_address = 0x00;
  }

  if ( last_address_written == last_sent_address ) { // Indica que se han enviado todos los datos faltantes
    currentMode = FINISH;
    return ;
  }

  read(&trama[3], TRAMA_DATA_SIZE, last_sent_address);    // Se almacenan en trama los datos de la flash

  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (loopTRANSFER) -> Calculando CRC");
  #endif
  uint16_t CRC = crc_calculate(trama);
  trama[trama_size-3] = (uint8_t)(CRC >> 8);
  trama[trama_size-2] = (uint8_t)(CRC & 0xFF);
  trama[trama_size-1] = 0x0A;               // STOP BYTE

  Serial1.write(trama, trama_size);         // Envío de trama
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (loopTRANSFER) -> Trama enviada correctamente, esperando ACK...");

  Serial.println("DEBUG (loopTRANSFER) -> Datos enviados: 0x");
  for ( uint8_t i = 0; i < TRAMA_COMM; i++ ) {
    Serial.print(trama[i], HEX);
    Serial.print(", 0x");
  }
  #endif


  ****************************************************************************************************
  *                                       Confirmation of data sent
  *************************************************************************************************** */
/* Se recibe el ACK/NACK del OBC
tiempo = millis();
while ( Serial1.available() < TRAMA_COMM ) {           // esperando ACK del OBC
  if ( tiempo >= timeOUT ) {
    // Serial1.write(nack_timeout_MUA_to_OBC, TRAMA_COMM);
    return ;
  }
}
// uint8_t len = Serial1.available();
// uint8_t recibido[len];
// Serial1.readBytes(recibido, len);

uint8_t recibido[TRAMA_COMM];
Serial1.readBytes(recibido, TRAMA_COMM);

#ifdef DEBUG_MAIN
Serial.println("DEBUG (loopTRANSFER) -> respuesta: 0x");
for ( uint8_t i = 0; i < TRAMA_COMM; i++ ) {
  Serial.print(recibido[i], HEX);
  Serial.print(", 0x");
}
#endif
* Si se va a buscar la trama en una serie de datos recibidos...
// uint8_t index_begin = 0;
// for ( uint8_t i = 0; i < len; i++ ) {
//   if ( recibido[i] == MISSION_ID ) {
//     index_begin = i;
//     break;
//   } else if ( i == len - 1 ) {
//     // Manejar, no se encontro id mission
//   }
// }

CRC = crc_calculate(recibido);       // TRAMA_COMM o index_begin
uint16_t crc_received = (recibido[TRAMA_COMM-3]<<8) | recibido[TRAMA_COMM-2];

if ( CRC != crc_received ) {
  Serial1.write(nack_MUA_to_OBC, TRAMA_COMM);     // invalid checksum NACK
  return ;
} else if ( recibido[0] != MISSION_ID ) {
  delay(timeOUT_invalid_frame);
  Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);  //  invalid frame NACK
  return ;
}

if ( recibido[1] == ACK_OBC_to_MUA ) {
  write_SENT_DATAaddress(last_sent_address + TRAMA_DATA_SIZE);  // se actualiza la direccion del ultimo dato enviado
} else if (recibido[1] == ID_FINISH) {
  currentMode = FINISH;
  return ;
} else if (recibido[1] == ID_TRANSFER_SYSINFO_MODE) {
  currentMode = TRANSFER_INFO_MODE;
  return ;
}
#endif
*/

// pio device monitor -p COM17