#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
// #include "max1932_driver.h"
// #include "calculos.h"
#include "obc_comm.h"

#define DEBUG_MAIN
#define P PA20 // Blink
#define PULSE PB08

// uint8_t status = 0;
float temperature = 0.0;
union FloatToUint32 { // para evitar aliasing y no violar las reglas del compilador
  float f;
  uint32_t u;
};
unsigned long time_ini = 0x00;
unsigned long timestamp = 0x00;

void setupCOUNT(void);
void loopCOUNT(void);
void setupTRANSFER(void);
void loopTRANSFER(void);

void setup() {
  delay(4000);

  Serial.begin(115200);                 // Puerto USB
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setup) -> Serial Iniciado");
  #endif

  requestOperationMode();               // Solicitud del modo de operación
  // currentMode = COUNT_MODE;

  if ( !start_flash() ) {               // Se utiliza en ambos modos de operación
    #ifdef DEBUG_MAIN
    Serial.println("DEBUG (setup) -> Comunicar al OBC que el flash no conecta.");
    #endif
  }

  switch ( currentMode ) {
    case COUNT_MODE:
      setupCOUNT();
      break;
    case TRANSFER_MODE:
      setupTRANSFER();
      break;
    default:
      /* Modo no seleccionado o incorrecto, manejar... */
      break;
  }
  Serial.println("Setup finalizado...");
}

void loop() {
  switch ( currentMode ) {
  case COUNT_MODE:
    loopCOUNT();
    break;
  case TRANSFER_MODE:
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
  setup_state = true;

  rtc.begin();
  getTimestampFromGPS();

  pinMode(PULSE_1, INPUT_PULLDOWN);
  // pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  // pinMode(SCL_Sensor, OUTPUT); // salida para Timer Control 2 
  pinMode(PA01, OUTPUT); // Salida para TC2 (utiliza también PA15)
  
  digitalWrite(P, LOW);

  activeInterrupt();

  setupTC2(); 
  // setupTC4();

  
  if ( !start_tmp100() ) { // Configuración del TMP100, HACER EN VARIOS INTENTOS
    #ifdef DEBUG_MAIN
    Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida");
    #endif
  }

  // erase_all();

  time_ini = millis();
  Serial.println("setupCount finalizado...");
  setup_state = true;

  /* Activar Placa Interfaz: 
  digitalWrite(Interface_EN, HIGH); */
}

void loopCOUNT(void) {
  if ( detect1 ) { // Se debe obtener el ancho del pulso...
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
  }

  /* Interrupción del TC2 cada 60 seg: Primeramente se deben desactivar las interrupciones de los pulso,
  luego se debe guardar en memoria -> timestamp(4B) - temperature(4B) - Vbias1y2(2x1B) - Count1y2(2x2B),
  realizar el algoritmo de polarización y obtener Vbias1y2, 
  inicializar las variables globales y activar las interrupciones de los pulsos. 
      El proceso se espera que tarde 5 segundos.*/
  if ( detect_TC ) {
    temperature = read_tmp100();
    FloatToUint32 temp;
    temp.f = temperature;

    if (isnan(temperature)) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (loopCOUNT) -> Error al leer la temperatura.");
      #endif
    }

    #ifdef DEBUG_MAIN
    Serial.print("TRAMA -> TMP: (");
    Serial.print(temp.f, 4);
    Serial.print(" ºC, 0x");
    Serial.print(temp.u, HEX); // Interpretado como entero sin signo de 32 bit
    Serial.print("), time: (");
    uint32_t tiempo = getTime();
    Serial.print(tiempo);
    Serial.print(", 0x");
    Serial.print(tiempo, HEX);
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
  //   if ( !write_mem((uint8_t *)&tiempo, sizeof(tiempo)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura tiempo.");
  //     #endif
  //   }
  //   if ( !write_mem((uint8_t *)&temp.u, sizeof(temperature)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura temperatura.");
  //     #endif
  //   }
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
  //   if ( !write_mem((uint8_t *)&pulse_count1, sizeof(pulse_count1)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count1.");
  //     #endif
  //   }
  //   if ( !write_mem((uint8_t *)&pulse_count2, sizeof(pulse_count2)) ) {
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (loopCOUNT) -> Fallo en la escritura pulse_count2.");
  //     #endif
  //   }

    detect_TC = false;
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
  unsigned long timeout = 1000;

  Serial1.begin(115200); // Establecer conexión
  #ifdef DEBUG_MAIN
  Serial.println("DEBUG (setupTRANSFER) -> Serial1 Iniciado");
  #endif

  // Obtener la cantidad de datos a ser transferida
  Serial1.write(0xC1); // Comando para solicitar cantidad de datos a transferir
  unsigned long start_time = millis();
  while ( Serial1.available() < sizeof(uint32_t) ) {
    if ( millis() - start_time > timeout ) {
      #ifdef DEBUG_MAIN
      Serial.println("DEBUG (setupTRANSFER) -> timeout esperando length_data - Serial1");
      #endif
      return;
    }
  }
  uint32_t length_data;
  Serial1.readBytes((uint8_t *)&length_data, sizeof(uint32_t));

  Serial.println("setupTRANSFER finalizado...");
  setup_state = true;
}

void loopTRANSFER(void) {
  // transferir datos
  // read_all();

  // revisar errores de transmisión

  // borrar datos transferidos de la memoria flash

  /* Al finalizar la transferencia, terminamos el proceso o hacemos conteos?? */
}