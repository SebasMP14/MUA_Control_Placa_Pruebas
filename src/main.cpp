/**
 * @file main.cpp
 * Control de la misión M.U.A. (Monitoring Unit of the South Anomaly)
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024-2025> (github)
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

#define DEBUG_MAIN
#define DEBUG_ // eliminar en todos lados para activar control de placa interfaz
#define MAX_ITER            5             // Protocol initialization attempts
// #define Elementos           400
#define Ventana             5             // Para Sliding Moving Average
#define OverVoltage         0.2238233 * 3             // Sobrevoltaje aplicado para la polarización de los SiPMs
#define Switching_Time_MAX  4             // Microseconds
#define TRAMA_DATA_SIZE     36            // Analizar
#define TRAMA_INFO_SIZE     36            // Analizar

// uint8_t status = 0;
uint8_t state = 0x01;                     // 
// uint32_t timestamp = 0;
uint8_t segundos = 60;                    // Calibración cada tantos segundos
uint8_t ACK_OBC_to_MUA = 0x04;
const float Voffset = 3.8;
const float ResisA = 1050;
const float ResisB = 2000;
float firstCurrent1 = 0.0;
float firstCurrent2 = 0.0;
float temperature1 = 0.0;
float temperature2 = 0.0;
float temperature = 0.0;
float searchMargin = 1.5;                 // 
const uint16_t Elementos = 400;
union FloatToUint32 {                     // Para evitar aliasing y no violar las reglas del compilador
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
unsigned long timestamp = 0x00;
float Lat = 0.0;
float Long = 0.0;
float Vbd1 = 0.0;                         // Breakdown Voltage Channel one
float Vbias1 = 0.0;                       // Polarization Voltage Channel one
float Vcurr1 = 0.0;                       // Breakdown Current Voltage Channel one
float Vbd2 = 0.0;                         // Breakdown Voltage Channel two
float Vbias2 = 0.0;                       // Polarization Voltage Channel two
float Vcurr2 = 0.0;                       // Breakdown Current Voltage Channel two

bool flag1 = false;
bool flag2 = false;

ADS1260 ads1260;

uint8_t sendTrama[TRAMA_DATA_SIZE] = {0x26};

uint16_t inverseVoltage_command[Elementos];   // Comandos a ser enviados a cada DAC
float inverseVoltage[Elementos];              // Voltages en el SiPM
float inverseVCurrent[Elementos];             // Corriente
float Filtered_voltage[Elementos];
float Filtered_current[Elementos];
float temperatureArray [Elementos];

void setupCOUNT(void);
void loopCOUNT(void);
void setupTRANSFER(void);
void loopTRANSFER(void);
void loopTRANSFERinfo(void);

void obtain_Curve_inverseVI(float Temperature, uint8_t CS_DAC, float REFERENCE);
float polarization_settling(float Vbd, uint8_t CS_DAC);

void setup() {
  delay(6000);

  Serial.begin(115200);                 // Puerto USB
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial Iniciado");
  #endif

  Serial.println("PRUEBA DE FLUJO DE PARTICULAS COMPENSANDO EL VBD...");

  Serial1.begin(9600);                // OBC (On Board Computer)
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial1 Iniciado");
  #endif

  // /* Inicialización de memoria Flash */
  if ( !start_flash() ) {              // Se utiliza en ambos modos de operación
    // break;
    #ifdef DEBUG_MAIN
    Serial.println("DEBUG (setup) -> Flash con problemss");
    #endif
  }
  delay(2000);

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
  get_OPstate(&state);

  pinMode(PULSE_1, INPUT_PULLDOWN);
  pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(PA01, OUTPUT);                            // Salida para TC2 (utiliza también PA15)
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_SiPM1, OUTPUT);
  pinMode(LED_SiPM2, OUTPUT);
  pinMode(INTERFACE_EN, OUTPUT);                    // 
  digitalWrite(INTERFACE_EN, LOW);

  Serial.print("Estado: 0x");
  Serial.println(state, HEX);

  switch ( state ) {
    case 0x00:                                // STAND_BY
    case 0xFF:
      currentMode = STAND_BY;
      requestOperationMode();                 // Espera del modo de operación
      if ( currentMode == COUNT_MODE ) {
        setupCOUNT();
      } else if ( currentMode == TRANSFER_DATA_MODE ) {
        setupTRANSFER();
      }
      break;

    case 0x01:                                // COUNT_MODE
      currentMode = COUNT_MODE;
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (setup) -> COUNT_MODE iniciado");
      #endif
      setupCOUNT();
      break;

    case 0x02:                                // TRANSFER_DATA_MODE
      currentMode = TRANSFER_DATA_MODE;
      setupTRANSFER();
      break;

    case 0x09:                                // TRANSFER_DATA_MODE
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

void loop() {
  switch ( currentMode ) {
    case STAND_BY:
      requestOperationMode();
      if (currentMode == COUNT_MODE) {
        setupCOUNT();
      } else if (currentMode == TRANSFER_DATA_MODE) {
        setupTRANSFER();
      } else if (currentMode == TRANSFER_INFO_MODE){
        setupTRANSFER();
      }
      break;

    case COUNT_MODE:
      loopCOUNT();
      #ifdef DEBUG__
      if ( Serial1.available() ) {
        requestOperationMode();
        if (currentMode == TRANSFER_DATA_MODE) {
          setupTRANSFER();
        } else if (currentMode == TRANSFER_INFO_MODE) {
          setupTRANSFER();
        }
      }
      #endif
      break;

    case TRANSFER_DATA_MODE:
      loopTRANSFER();
      break;

    case TRANSFER_INFO_MODE:
      loopTRANSFERinfo();
      break;
    
    case FINISH:
      
      break;

    default:
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loop) -> UNKNOWN_MODE");
      #endif
      delay(2000);
      requestOperationMode();
      if ( !setup_state && currentMode != UNKNOWN_MODE) {
        setupCOUNT();
      }
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
void setupCOUNT(void) {
  #ifdef DEBUG_
  SPI.begin();                                      // BUS MAX1y2 & DAC1y2
  Wire.begin();                                     // BUS TMP100 y MCP4561
  Wire.setClock(400000);
  #endif

  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( rtc.begin() ) {                            // Configuración del RTC
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
  // getTimestampFromGPS();                            // 

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_SiPM1, LOW);
  digitalWrite(LED_SiPM2, LOW);
  digitalWrite(INTERFACE_EN, HIGH);                 // Activación de Placa Interfaz(5V-3V3) y ADC1260(5V)

  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> writing in MCP ");
  #endif
  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( writeMCP(0x9C) ) {                         // Configuración del MCP4561
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de MCP4561 exitosa.");
      #endif
      break;
    } else {
      #ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de MCP4561 fallida: ");
      Serial.println(iter_counter);
      #endif
      delay(10);
    }
  }
  Serial.print("MCP escrito en: ");
  Serial.println(readMCP(), HEX);

  delay(START_UP_TIME_ADS);                         // Habilitación del ADC (REVISAR TIEMPO)
  
  start_dac8551(SPI_CS_DAC1);                       // SiPM 1
  start_max1932(SPI_CS_MAX1);

  start_dac8551(SPI_CS_DAC2);                       // SiPM 2
  start_max1932(SPI_CS_MAX2);

  #ifdef DEBUG_
  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( start_tmp100() ) {                         // Configuración del TMP100
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 exitosa.");
      #endif
      break;
    } else {
      #ifdef DEBUG_MAIN
      Serial.print("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida: ");
      Serial.println(iter_counter);
      #endif
      delay(10);
    }
  }
  #endif

  #ifdef DEBUG_
  // ADC Configuration
  ads1260.begin();
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   // respuesta esperada: 00100100
  #endif
  ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100);           // 40 KSPS - FIR (Page 30)
  // ads1260.writeRegisterData(ADS1260_MODE0, 0b01101100);           // 14400 SPS
  delay(50);
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (setupCOUNT) -> MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   
  Serial.print("DEBUG (setupCOUNT) -> PGA: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
  #endif
  // ads1260.writeRegisterData(ADS1260_PGA, 0b10000000);             // BYPASS MODE
  // #ifdef DEBUG_MAIN
  // Serial.print("DEBUG (setupCOUNT) -> PGA BYPASS MODE: ");
  // Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
  // #endif
  ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000);           // STATENB  REVISARRRRRRRRRRRRRRRRRRRRRRRR
  // ads1260.writeRegisterData(ADS1260_REF, 0b00010000);             // REF 2.498V ENABLE
  delay(300);

  // while (1) {
  //   delay(500);
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   delay(500);
  //   digitalWrite(LED_BUILTIN, LOW);
  //   delay(START_UP_TIME_ADS);
  //   ads1260.begin();
  //   Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);
  // }

  // erase_all();

  external_ref = ads1260.readRef();                             // Se lee la referencia
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (loopCOUNT) -> readRef: ");
  Serial.println(external_ref, 6);
  #endif
  // Primera polarización de los SiPMs
  // Channel 1
  write_dac8551_reg(0x7FFF, SPI_CS_DAC1);                       // Activación de Vout1 al mínimo valor
  write_max_reg(0x01, SPI_CS_MAX1);
  temperature1 = read_tmp100();
  obtain_Curve_inverseVI(temperature1, SPI_CS_DAC1, external_ref);
  sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage);
  sliding_moving_average(inverseVCurrent, Elementos, Ventana, Filtered_current);
  Vbd1 = obtain_Vbd(Filtered_current, Filtered_voltage, Elementos, &Vcurr1);
  Vbias1 = polarization_settling(Vbd1, SPI_CS_DAC1);
  activeInterrupt1();                                           // Una vez polarizado
  flag1 = true;

  #ifdef DEBUG__
  // Channel 2
  write_dac8551_reg(0x7FFF, SPI_CS_DAC2);                       // Activación de Vout2 al mínimo valor
  write_max_reg(0x01, SPI_CS_MAX2);
  temperature2 = read_tmp100();
  obtain_Curve_inverseVI(temperature2, SPI_CS_DAC2, external_ref);
  sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage);
  sliding_moving_average(inverseVCurrent, Elementos, Ventana, Filtered_current);
  Vbd2 = obtain_Vbd(Filtered_current, Filtered_voltage, Elementos, &Vcurr2);
  Vbias2 = polarization_settling(Vbd2, SPI_CS_DAC2);
  activeInterrupt2();                                           // Una vez polarizado
  #endif

  #endif
  setupTC2(segundos);

  setup_state = true;
  #ifdef DEBUG_MAIN
  Serial.println("setupCount finalizado...");
  #endif
  time_ini = millis();

}

void loopCOUNT(void) {
  if ( detect1 ) {                            // Se puede obtener el ancho del pulso?, es necesario?: (si, no)
    detect1 = false;
    #ifdef DEBUG_MAIN_
    Serial.print("COUNT1: ");
    Serial.println(pulse_count1);
    #endif
  }

  if ( detect2 ) {
    detect2 = false;
    #ifdef DEBUG_MAIN_
    Serial.print("COUNT2: ");
    Serial.print(pulse_count2);
    #endif
  }

  if ( (millis() - time_ini) >= 5000 ) {      /////////// Rutina de prueba
    // read_all();
    digitalWrite(LED_BUILTIN, HIGH);                      // Blink
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("millis: ");
    time_ini = millis();
    Serial.print(time_ini);
    Serial.print(", timestamp: ");
    Serial.print(rtc.now().unixtime());
    Serial.print(", Temperatura: ");
    #ifdef DEBUG_
    Serial.println(read_tmp100(), 4);
    #endif
  }

  /**   Interrupción del TC2 cada 60 seg: Primeramente se deben desactivar las interrupciones de los pulsos,
   * luego se debe guardar en memoria -> 
   * timestamp(4B) - LatyLong(2x4B) - temperature(4B) - Count1y2(2x2B) - Vbias1y2(2x4B) - Vcurr1y2(2x4B),
   * realizar el algoritmo de polarización y obtener Vbias1y2, inicializar las variables globales y activar 
   * las interrupciones de los pulsos.
   *    El proceso se espera que tarde 5 segundos por SiPM y la calibración se hará individualmente. */
  if ( detect_TC ) {
    detect_TC = false;
    disableTC2();
    desactiveInterrupt1();
    desactiveInterrupt2();

    temperature = (temperature1 );//+ temperature2) / 2;
    
    timestamp = getTime();
    lati.f = Lat;
    longi.f = Long;
    temp.f = temperature;
    vbd1.f = Vbd1;
    vbd2.f = Vbd2;
    vcurr1.f = Vcurr1;
    vcurr2.f = Vcurr2;

    if (isnan(temperature)) {
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
    Serial.print(vbd1.u, HEX); // hacer FloatTOUINT
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
    Serial.print(")");
    #endif

    #ifdef DEBUG__
    read_all();
    #endif
    
    /* GUARDADO Little-Endian */
    if ( !write_mem((uint8_t *)&timestamp, sizeof(timestamp)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura timestamp.");
      #endif
    }
    if ( !write_mem((uint8_t *)&lati.u, sizeof(lati.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Lat.");
      #endif
    }
    if ( !write_mem((uint8_t *)&longi.u, sizeof(longi.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Long.");
      #endif
    }
    if ( !write_mem((uint8_t *)&temp.u, sizeof(temperature)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura temperatura.");
      #endif
    }
    if ( !write_mem((uint8_t *)&pulse_count1, sizeof(pulse_count1)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count1.");
      #endif
    }
    if ( !write_mem((uint8_t *)&pulse_count2, sizeof(pulse_count2)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count2.");
      #endif
    }
    if ( !write_mem((uint8_t *)&vbd1.u, sizeof(vbd1.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vbd1.");
      #endif
    }
    if ( !write_mem((uint8_t *)&vbd2.u, sizeof(vbd2.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vbd2.");
      #endif
    }
    if ( !write_mem((uint8_t *)&vcurr1.u, sizeof(vcurr1.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vcurr1.");
      #endif
    }
    if ( !write_mem((uint8_t *)&vcurr2.u, sizeof(vcurr2.u)) ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vcurr2.");
      #endif
    }

    #ifdef DEBUG_
    external_ref = ads1260.readRef();                             // Se lee nuevamente la ref del ADC

    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loopCOUNT) -> readRef: ");
    Serial.println(external_ref, 6);
    #endif

    // Channel 1 Polarization
    write_dac8551_reg(0x7FFF, SPI_CS_DAC1);                       // Vout1 al mínimo valor
    write_max_reg(0x01, SPI_CS_MAX1);
    temperature1 = read_tmp100();
    obtain_Curve_inverseVI(temperature1, SPI_CS_DAC1, external_ref);
    sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage); // Voltage Filtering 
    sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage); // Current Filtering
    Vbd1 = obtain_Vbd(Filtered_current, Filtered_voltage, Elementos, &Vcurr1);    // 
    Vbias1 = polarization_settling(Vbd1, SPI_CS_DAC1);
    activeInterrupt1();
    flag1 = true;

    #ifdef DEBUG__
    // Channel 2 Polarization
    write_dac8551_reg(0x7FFF, SPI_CS_DAC2);                       // Vout2 al mínimo valor
    write_max_reg(0x01, SPI_CS_MAX2);
    temperature2 = read_tmp100();
    obtain_Curve_inverseVI(temperature2, SPI_CS_DAC2, external_ref);
    sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage);
    sliding_moving_average(inverseVoltage, Elementos, Ventana, Filtered_voltage);
    Vbd2 = obtain_Vbd(Filtered_current, Filtered_voltage, Elementos, &Vcurr2);
    Vbias2 = polarization_settling(Vbd2, SPI_CS_DAC2);
    activeInterrupt2();
    #endif

    #endif
    enableTC2();
  }

  ///////** Para imprimir los puntos */
  if ( flag1 && !detect_TC ) {
    flag1 = false;

    Serial.println("-------------------- DATOS OBTENIDOS --------------------");
    Serial.println("i, Voltage, VCorriente, T, (V*12)-3.8-k(VCorr-firstCurr1), (VCorr-firstCurr1)/2000");
    for ( uint16_t i = 1; i < Elementos; i++ ) {
      Serial.print(i);
      Serial.print(",");
      Serial.print(inverseVoltage[i], 7);
      Serial.print(",");
      Serial.print(inverseVCurrent[i], 7);
      Serial.print(",");
      Serial.print(temperatureArray[i]);
      Serial.print(",");
      Serial.print((inverseVoltage[i]*12)-3.8-(ResisA*(inverseVCurrent[i]-firstCurrent1)/ResisB), 7);
      Serial.print(",");
      Serial.println((inverseVCurrent[i]-firstCurrent1)/ResisB, 10);
    }
    Serial.print("Vbd Calculado: ");
    Serial.println(Vbd1, 6);
    Serial.print("Vbias seteado: ");
    Serial.println(Vbias1, 6);
  }

}

/******************************************************
 * @fn      setupTRANSFER - loopTRANSFER
 * @brief   - Setup: Inicializaciones del modo de operación: Transferencia de Datos
 *          - Loop: Se transfieren los datos 
 * @param   NONE
 * @return  NONE
 * TODO: - Se debe determinar cuantos Bytes se van a transferir al OBC
 */
void setupTRANSFER(void) {
  // unsigned long timeOUT = 1000;

  // Serial1.begin(115200); // Establecer conexión
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setupTRANSFER) -> Serial1 Iniciado");
  #endif

  // write_OPstate(*state);
  disableTC2(); // Funka
  desactiveInterrupt1();
  desactiveInterrupt2();
  digitalWrite(INTERFACE_EN, LOW);

  
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setupTRANSFER) -> setupTRANSFER finalizado...");
  #endif
  setup_state = true;
}

void loopTRANSFER(void) {
  // transferir datos
  // read_all();
  delay(1000);
  /*
  * Esta sección prueba el envío de los datos al OBC, los únicos datos enviados son los que contiene "trama"
  * y se envía X veces 
  */
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

  #ifdef DEBUG_
  uint32_t last_address_written = 0xFFFFFFFF;
  get_address(&last_address_written);

  uint8_t trama_size = TRAMA_DATA_SIZE + TRAMA_COMM;
  uint8_t trama[trama_size] = {MISSION_ID, ID_TRANSFER_MODE, TRAMA_DATA_SIZE};
  uint32_t last_sent_address = 0xFFFFFFFF;
  // if ( get_SENT_DATAaddress(&last_sent_address) && last_sent_address == 0xFFFFFFFF ) { // Primera vez en enviar datos         
  //   last_sent_address = 0x00;
  // }

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
  Serial.println("DEBUG (loopTRANSFER) -> Datos enviados: 0x");
  for ( uint8_t i = 0; i < TRAMA_COMM; i++ ) {
    Serial.print(trama[i], HEX);
    Serial.print(", 0x");
  }
  #endif

  /* Se recibe el ACK/NACK del OBC */

  while ( Serial1.available() < TRAMA_COMM );           // esperando ACK de OBC
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
  /* Si se va a buscar la trama en una serie de datos recibidos... */
  // uint8_t index_begin = 0;
  // for ( uint8_t i = 0; i < len; i++ ) { 
  //   if ( recibido[i] == MISSION_ID ) {
  //     index_begin = i;
  //     break;
  //   } else if ( i == len - 1 ) {
  //     // Manejar, no se encontro id mission
  //   }
  // }

  CRC = crc_calculate(&recibido[TRAMA_COMM]);       // TRAMA_COMM o index_begin
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
    write_SENT_DATAaddress(last_sent_address + TRAMA_DATA_SIZE);
  } else {
    return ;
  }
  #endif

  /* Al finalizar la transferencia, terminamos el proceso */
}

void loopTRANSFERinfo(void) {

}

/************************************************************************************************************
 * @fn      obtain_Curve_inverseVI
 * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
 *          ADC.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @param   CS_DAC: Selección del Canal
 * @return  ---todo
 */
void obtain_Curve_inverseVI(float Temperature, uint8_t CS_DAC, float REFERENCE) {
  uint8_t muxP0, muxP1, led;                   // Configuración de lectura: Canal 1 o 2
  muxP0 = ADS1260_MUXP_AIN0;
  muxP1 = ADS1260_MUXP_AIN2;
  led = LED_SiPM1;
  if ( CS_DAC == SPI_CS_DAC2 ) {
    muxP0 = ADS1260_MUXP_AIN1;
    muxP1 = ADS1260_MUXP_AIN3;
    led = LED_SiPM2;
  }

  float Vbd_Teo = Vbd_teorical(Temperature);
  #ifdef DEBUG_MAIN
  Serial.print("Vbd teorico: ");
  Serial.println(Vbd_Teo, 6);
  #endif
  float Vlimite_inferior = max(24.588, Vbd_Teo + Voffset - searchMargin);  // 24.588 es el minimo valor a la salida del MAX
  float Vlimite_superior = min(36, Vbd_Teo + Voffset + searchMargin);      // 36 es el máximo valor a la salida del MAX
  uint16_t Vlim_inf = VDAC_command(Vlimite_inferior); // Comando
  uint16_t Vlim_sup = VDAC_command(Vlimite_superior); // Comando
  uint16_t paso = (uint16_t)(Vlim_inf - Vlim_sup) / Elementos;
  unsigned long start_time, total_time;

  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (obtain_Curve_inverseVI) -> Limites (dec, hex): ");
  Serial.print(out_voltage(Vlim_inf), 4);
  Serial.print(", ");
  Serial.print(out_voltage(Vlim_sup), 4);
  Serial.print(", ");
  Serial.print(Vlim_inf, HEX);
  Serial.print(", ");
  Serial.print(Vlim_sup, HEX);
  Serial.print(", paso: ");
  Serial.println(paso);
  #endif
  digitalWrite(led, HIGH);

  start_time = micros();
  for ( uint16_t i = 0; i < Elementos; i++ ) {
    inverseVoltage_command[i] = Vlim_inf - i * paso;          // Comandos para el DAC
    write_dac8551_reg(inverseVoltage_command[i], CS_DAC);
    delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    delay(5); // Settling time of the MAX (Vout)
    // Considerar el tiempo de asentamiento del filtro pasa bajos de 2ndo orden...

    inverseVoltage[i] = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), REFERENCE);
    inverseVCurrent[i] = ads1260.computeVolts(ads1260.readData(muxP1, ADS1260_MUXN_AINCOM), REFERENCE);
    temperatureArray[i] = read_tmp100();

    /*  Si se decide implementar un submuestreo  */
    // uint32_t aux = 0x00;
    // ads1260.connectMUX(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM);
    // for ( uint8_t j = 0; j < 10; j++ ) {
    //   aux += ads1260.readConversion();
    //   delayMicroseconds(100);
    // }
    // inverseVoltage[i] = ads1260.computeVolts((uint32_t)(aux/10));
    // aux = 0x00;
    // ads1260.connectMUX(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM);
    // for ( uint8_t j = 0; j < 10; j++ ) {
    //   aux += ads1260.readConversion();
    //   delayMicroseconds(100);
    // }
    // inverseVCurrent[i] = ads1260.computeVolts((uint32_t)(aux/10));
  }
  total_time = micros() - start_time;
  write_dac8551_reg(0x7FFF, CS_DAC);
  digitalWrite(led, LOW);
  firstCurrent1 = inverseVCurrent[0];
  float Ts = (float)total_time / (Elementos * 1000);
  #ifdef DEBUG_MAIN
  Serial.print("First VCurrent: ");
  Serial.println(firstCurrent1, 7);
  Serial.print("Tiempo promedio de muestreo [ms]: ");
  Serial.println(Ts, 4);
  #endif
}

/************************************************************************************************************
 * @fn      polarization_settling
 * @brief   Establecimiento del voltaje de polarización de un canal
 * @param   Vbd:
 * @param   CS_DAC: 
 * @return  Vbias
 */
float polarization_settling(float Vbd, uint8_t CS_DAC) {
  uint8_t muxP0 = ADS1260_MUXP_AIN0;// muxP1;                   // Configuración de lectura: Canal 1 o 2
  float Vbias = 0.0;
  if ( CS_DAC == SPI_CS_DAC1 ) {
    muxP0 = ADS1260_MUXP_AIN0;
    // muxP1 = ADS1260_MUXP_AIN2;
  } else if ( CS_DAC == SPI_CS_DAC2 ) {
    muxP0 = ADS1260_MUXP_AIN1;
    // muxP1 = ADS1260_MUXP_AIN3;
  }

  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (polarization_settling) -> iniciando");
  #endif

  uint16_t i = 0x01;
  uint16_t command = VDAC_command(Vbd + 0.95*OverVoltage);
  uint16_t newCommand;
  while ( Vbias < Vbd + (OverVoltage) ) { 
    newCommand = command - i;
    write_dac8551_reg(newCommand, CS_DAC);       // Disminuir el comando -> aumento de Vout
    i = i + 10; // paso de 10
    
    delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    delay(5); // Settling time of the MAX (Vout)
    // Considerar el tiempo de asentamiento del filtro pasa bajos de 2ndo orden... // 354.96 useg

    Vbias = ads1260.computeVolts(ads1260.readData(muxP0, ADS1260_MUXN_AINCOM), external_ref);

    if ( newCommand < 0x000F ) {                // evitamos underflow
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (polarization_settling) -> Limit reached");
      #endif
      break;
    }
  }

  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (polarization_settling) -> pasos: ");
  Serial.println(i);
  Serial.print("DEBUG (polarization_settling) -> Vbias: ");
  Serial.println(Vbias, 6);
  #endif

  return Vbias;
}




// pio device monitor -p COM17