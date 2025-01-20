/**
 * main.cpp
 * Control de la misión M.U.A, ...
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
#include "max1932_driver.h"
#include "dac8551_driver.h"
#include "calculos.h"
#include "obc_comm.h"

#define DEBUG_MAIN
#define MAX_ITER 10
#define Elementos 100
#define OverVoltage 3                     // Sobrevoltaje aplicado para la polarización de los SiPMs
#define Switching_Time_MAX 4              // Microseconds
#define P LED_BUILTIN // Blink
#define PULSE PB08

// uint8_t status = 0;
float temperature = 0.0;
union FloatToUint32 {                     // Para evitar aliasing y no violar las reglas del compilador
  float f;
  uint32_t u;
};
unsigned long time_ini = 0x00;
unsigned long timestamp = 0x00;
uint8_t ACK_OBC_to_MUA = 0x04;


Adafruit_ADS1115 ads;
float inverseVoltage[Elementos];          // Tensión inversa aplicada al SiPM para obtener "inverseCurrent_I"
float inverseCurrent_V[Elementos];        // Tensión leída, correspondiente a la corriente inversa
float inverseCurrent_I[Elementos];        // Corriente inversa, convertida de "inverseCurrent_V[]"

uint8_t sendTrama[TRAMA_SIZE] = {0x03};

void setupCOUNT(void);
void loopCOUNT(void);
void setupTRANSFER(void);
void loopTRANSFER(void);

void obtain_Curve_inverseVI(float Temperature);

void setup() {
  delay(4000);

  Serial.begin(115200);                 // Puerto USB
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial Iniciado");
  #endif

  Serial1.begin(115200);                // OBC
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial1 Iniciado");
  #endif

  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( start_flash() ) {               // Se utiliza en ambos modos de operación
      break;
    }
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (setup) -> La flash no conecta, intento: ");
    Serial.println(iter_counter);
    #endif
  }

  // Restaurar último estado guardado en memoria
  // if ( erase_all() ) {
  //   Serial.println("Flash borrada");
  // }
  // write_OPstate(0x00);
  uint8_t state = read_OPstate();
  switch ( state ) {
    case 0x00:          // STAND_BY
    case 0xFF:
      currentMode = STAND_BY;
      requestOperationMode();               // Espera del modo de operación
      if ( currentMode == COUNT_MODE ) {
        setupCOUNT();
      } else if ( currentMode == TRANSFER_DATA_MODE ) {
        setupTRANSFER();
      }
      break;

    case 0x01:          // COUNT_MODE
      currentMode = COUNT_MODE;
      setupCOUNT();
      break;

    case 0x02:          // TRANSFER_DATA_MODE
      currentMode = TRANSFER_DATA_MODE;
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
      }
      break;
    case COUNT_MODE:
      loopCOUNT();
      if ( Serial1.available() ) {
        requestOperationMode();
        if (currentMode == TRANSFER_DATA_MODE) {
          setupTRANSFER();
        }
      }
      break;
    case TRANSFER_DATA_MODE:
      loopTRANSFER();
      break;
    default:
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loop) -> UNKNOWN_MODE");
      #endif
      delay(5000);
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
 * TODO: - Se debe establecer la tasa de cambio de la temperatura para ajustar el tiempo
 *        interrupcion del timer counter
 */
void setupCOUNT(void) {

  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( rtc.begin() ) { // Configuración del RTC, HACER EN VARIOS INTENTOS
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
  getTimestampFromGPS();

  pinMode(PULSE_1, INPUT_PULLDOWN);
  // pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  pinMode(PA01, OUTPUT); // Salida para TC2 (utiliza también PA15)
  
  digitalWrite(P, LOW);

  activeInterrupt();

  setupTC2();
  // setupTC4();

  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
    if ( start_tmp100() ) { // Configuración del TMP100, HACER EN VARIOS INTENTOS
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

  // ads.begin();

  // erase_all();

  setup_state = true;
  Serial.println("setupCount finalizado...");
  time_ini = millis();

  /* Activar Placa Interfaz: 
  digitalWrite(Interface_EN, HIGH); */
}

void loopCOUNT(void) {
  if ( detect1 ) {                  // Se debe obtener el ancho del pulso?
    detect1 = false;
    #ifdef DEBUG_MAIN
    Serial.print("COUNT1: ");
    Serial.print(pulse_count1);
    Serial.print(", Duración: ");
    Serial.print(pulse_width);
    Serial.println(" ticks");
    #endif
  }

  if ( detect2 ) {
    detect2 = false;
    #ifdef DEBUG_MAIN
    Serial.print("COUNT2: ");
    Serial.print(pulse_count2);
    Serial.print(", Duración: ");
    Serial.print(pulse_width);
    Serial.println(" ticks");
    #endif
  }

  if ( (millis() - time_ini) >= 5000 ) {      /////////// Rutina de prueba
    // read_all();
    digitalWrite(P, HIGH); // Blink
    delay(100);
    digitalWrite(P, LOW);
    Serial.print("millis: ");
    time_ini = millis();
    Serial.println(time_ini);
    Serial.print("timestamp: ");
    Serial.println(rtc.now().unixtime());
    Serial.print("Temperatura: ");
    Serial.println(read_tmp100(), 4);
  }

  /**   Interrupción del TC2 cada 60 seg: Primeramente se deben desactivar las interrupciones de los pulso,
   * luego se debe guardar en memoria -> timestamp(4B) - temperature(4B) - Vbias1y2(2x1B) - Count1y2(2x2B),
   * realizar el algoritmo de polarización y obtener Vbias1y2, 
   * inicializar las variables globales y activar las interrupciones de los pulsos. 
   *    El proceso se espera que tarde 5 segundos. */
  if ( detect_TC ) {
    desactiveInterrupt();
    temperature = read_tmp100();
    FloatToUint32 temp;
    temp.f = temperature;

    if (isnan(temperature)) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Error al leer la temperatura.");
      #endif
    }

    #ifdef DEBUG_MAIN
    Serial.print("TRAMA -> time: (");
    uint32_t tiempo = getTime();
    Serial.print(tiempo);
    Serial.print(", 0x");
    Serial.print(tiempo, HEX);
    Serial.print("), TMP: (");
    Serial.print(temp.f, 4);
    Serial.print(" ºC, 0x");
    Serial.print(temp.u, HEX); // Interpretado como entero sin signo de 32 bit
    Serial.print("), Vbr1: (), Vbr2: (), Count1: (");
    Serial.print(pulse_count1);
    Serial.print(", 0x");
    Serial.print(pulse_count1, HEX);
    Serial.print("), Count2: (");
    Serial.print(pulse_count2);
    Serial.print(", 0x");
    Serial.print(pulse_count2, HEX);
    Serial.println(")");
    #endif

    read_all();
    
    /* GUARDADO Little-Endian */
    // if ( !write_mem((uint8_t *)&tiempo, sizeof(tiempo)) ) {
    //   #ifdef DEBUG_MAIN
    //   Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura tiempo.");
    //   #endif
    // }
    // if ( !write_mem((uint8_t *)&temp.u, sizeof(temperature)) ) {
    //   #ifdef DEBUG_MAIN
    //   Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura temperatura.");
    //   #endif
    // }
  //   if ( !write_mem((uint8_t *)&Vbr1, sizeof(Vbr1)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vbr1.");
  //     #endif
  //   }
  //   if ( !write_mem((uint8_t *)&Vbr2, sizeof(Vbr2)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura Vbr2.");
  //     #endif
  //   }
    // if ( !write_mem((uint8_t *)&pulse_count1, sizeof(pulse_count1)) ) {
    //   #ifdef DEBUG_MAIN
    //   Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count1.");
    //   #endif
    // }
    // if ( !write_mem((uint8_t *)&pulse_count2, sizeof(pulse_count2)) ) {
    //   #ifdef DEBUG_MAIN
    //   Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count2.");
    //   #endif
    // }

    

    detect_TC = false;
    activeInterrupt();
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

  

  Serial.println("setupTRANSFER finalizado...");
  setup_state = true;
}

void loopTRANSFER(void) {
  // transferir datos
  // read_all();
  delay(1000);
  uint8_t data_size = 10;
  uint8_t data[data_size];
  read_until(data, data_size);
  for ( uint8_t i = 1; i < data_size+2; i++ ) {
    if ( i == 1 ) {
      sendTrama[i] = data_size;
    } else {
      sendTrama[i] = data[i-2];
    }
  }
  #ifdef DEBUG_MAIN
  Serial.println("Calculando CRC");
  #endif
  uint16_t CRC = calcularCRC(sendTrama, TRAMA_SIZE-2);
  sendTrama[TRAMA_SIZE-2] = (CRC >> 8) & 0xFF;  // CRCH
  sendTrama[TRAMA_SIZE-1] =  CRC & 0xFF;        // CRCL
  Serial1.write(sendTrama, TRAMA_SIZE);
  #ifdef DEBUG_MAIN
  Serial.println("Datos enviados");
  #endif

  while ( Serial1.available() != 1 ); // esperando ACK de OBC
  uint8_t recibido;
  Serial1.readBytes(&recibido, 1);
  #ifdef DEBUG_MAIN
  Serial.println("respuesta: 0x");
  Serial.print(recibido, HEX);
  #endif

  if ( recibido == ACK_OBC_to_MUA ) {
    #ifdef DEBUG_MAIN
    Serial.println("ACK recibido con exito");
    #endif
    currentMode = STAND_BY;
    write_OPstate(0x00);
  } else {
    #ifdef DEBUG_MAIN
    Serial.println("Transmisión fallida");
    #endif
  }

  // borrar datos transferidos de la memoria flash, no, mantener...

  /* Al finalizar la transferencia, terminamos el proceso */
}

/************************************************************************************************************
 * @fn      obtain_Curve_inverseVI
 * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
 *          ADC.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @return  ---todo
 */
void obtain_Curve_inverseVI(float Temperature) {
  float Vbd_Teo = Vbd_teorical(Temperature);
  float Vlimite_inferior = Vbd_Teo - 6;
  float Vlimite_superior = Vbd_Teo + 4;
  float Vbarrido = Vlimite_inferior;
  float paso = (Vlimite_superior - Vlimite_superior) / Elementos;

  for (int i = 0; i < Elementos; i++) {
    write_max_reg(VMax_command(Vbarrido));
    delayMicroseconds(Switching_Time_MAX);
    // Aquí se debe validar la tensión seteada en la salida del max con el ADC
    // inverseVoltage[i] = ads.computeVolts(ads.readADC_SingleEnded(0));
    inverseCurrent_V[i] = ads.computeVolts(ads.readADC_SingleEnded(1));
    // convertir a los valores de corriente correspondientes en "inverseCurrent_I"
    // inverseCurrent_I[i] = ?;
    Vbarrido += paso;
  }
  
  // float Vbias = obtain_Vbd(inverseCurrent_I, inverseVoltage, Elementos) + OverVoltage;
}




// pio device monitor -p COM17