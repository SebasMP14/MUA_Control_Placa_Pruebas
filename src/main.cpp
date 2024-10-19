#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
// #include "max1932_driver.h"
// #include "calculos.h"

#define DEBUG_MAIN
#define P PA20 // Blink
#define PULSE PB08

// uint8_t status = 0;
float temperature = 0.0;
union FloatToUint32 { // para evitar aliasing y no violar las reglas del compilador
  float f;
  uint32_t u;
};
unsigned long time_ini = 0x0;
enum OperationMode {
    COUNT_MODE,
    TRANSFER_MODE,
    UNKNOWN_MODE
};
OperationMode currentMode = UNKNOWN_MODE;
unsigned long timestamp = 0;
bool setup_state = false;

void requestOperationMode(void);
void getTimestampFromGPS(void);
void setupCOUNT(void);
void loopCOUNT(void);
void setupTRANSFER(void);
void loopTRANSFER(void);

void setup() {
  delay(4000);

  Serial.begin(115200);                 // USB
  Serial.println("Serial Iniciado");
  Serial1.begin(9600);                // OBC
  Serial.println("Serial1 Iniciado");

  requestOperationMode();
  // currentMode = COUNT_MODE;

  if ( !start_flash() ) { // Se utiliza en ambos modos
    #ifdef DEBUG_MAIN
    Serial.println("Comunicar al OBC que el flash no conecta.");
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
    Serial.println("DEBUG (loop): UNKNOWN_MODE");
    delay(5000);
    requestOperationMode();
    if ( !setup_state && currentMode != UNKNOWN_MODE) {
      setupCOUNT();
    }
    /* Modo no seleccionado o incorrecto, manejar... */
    break;
  }
}

void setupCOUNT(void) {
  setup_state = true;

  Serial2.begin(9600);                // GPS
  Serial.println("Serial2 Iniciado");
  getTimestampFromGPS();

  pinMode(PULSE_1, INPUT_PULLDOWN);
  // pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  // pinMode(SCL_Sensor, OUTPUT); // salida para Timer Control 2 
  // pinMode(PA01, OUTPUT); // Salida para TC2 (utiliza también PA15)
  
  digitalWrite(P, LOW);

  attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1, RISING);
  // attachInterrupt(digitalPinToInterrupt(PULSE), handlePulse1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PULSE), handlePulse1_FALLING, FALLING);
  setupTC2(); 
  // setupTC4();

  // Configuración del TMP100
  if ( !start_tmp100() ) {
    #ifdef DEBUG_MAIN
    Serial.println("DEBUG (setupCOUNT): Inicialización de TMP100 fallida");
    #endif
  }

  // erase_all();

  time_ini = millis();
  Serial.println("setupCount finalizado...");
}

void loopCOUNT(void) {

  if ( detect1 ) {
    #ifdef DEBUG_MAIN
    Serial.print("Pulsos: ");
    Serial.print(pulse_count1);
    Serial.print(", ");
    // Serial.println(pulse_count2);
    Serial.print("Duración: ");
    Serial.print(pulse_Width);
    Serial.println(" ticks");
    #endif
    detect1 = false;
    // detect2 = false;
  }

  if ( (millis() - time_ini) >= 5000 ) {
    // read_all();
    digitalWrite(P, HIGH); // Blink
    delay(100);
    digitalWrite(P, LOW);
    Serial.print("millis: ");
    time_ini = millis();
    Serial.println(time_ini);
  }

  if ( detect_TC ) {
    temperature = read_tmp100();
    FloatToUint32 temp;
    temp.f = temperature;
    if (isnan(temperature)) {
      #ifdef DEBUG_MAIN
      Serial.println("Error al leer la temperatura.");
      #endif
    } else {
      Serial.print("TMP: ");
      Serial.print(temperature, 4);
      Serial.print(" ºC, 0x");
      Serial.print(temp.u, HEX); // Interpretado como entero sin signo de 32 bit
      // Serial.print(*(unsigned long *)&temperature, HEX);
      Serial.print(", time: ");
      uint32_t tiempo = millis();
      Serial.print(tiempo);
      Serial.print(", 0x");
      Serial.print(tiempo, HEX);
      Serial.print(", ");
      Serial.println();

      /* GUARDADO Little-Endian */
    //   if ( !write_mem((uint8_t *)&tiempo, sizeof(tiempo)) ) {
    //     #ifdef DEBUG_MAIN
    //     Serial.println("Fallo en la escritura.");
    //     #endif
    //   }
    //   if ( !write_mem((uint8_t *)&temp.u, sizeof(temperature)) ) {
    //     #ifdef DEBUG_MAIN
    //     Serial.println("Fallo en la escritura.");
    //     #endif
    //   }
    }
    // read_all();
    digitalWrite(P, HIGH);
    delay(200);
    digitalWrite(P, LOW);
    // delay(100);
    detect_TC = false;
  }

}

void setupTRANSFER(void) {

}

void loopTRANSFER(void) {


}

void requestOperationMode(void) {
  Serial1.write(0xAA);  // Comando para solicitar el modo
  
  while (Serial1.available() < 0) ;
  delay(100);
  uint8_t response;
  Serial1.readBytes(&response, 1); // TRAMA del OBC
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (requestOperationMode): Recibido de Serial1 0x");
  Serial.println(response, HEX);
  #endif
  
  if (response == 0xA1) { // Decodificar la respuesta y establecer el modo
    currentMode = COUNT_MODE;
  } else if (response == 0xA4) {
    currentMode = TRANSFER_MODE;
  } else {
    currentMode = UNKNOWN_MODE;
  }
}

void getTimestampFromGPS(void) {
  Serial2.write(0xBB);  // Comando para solicitar el timestamp
  
  while ( Serial2.available() < sizeof(uint32_t) ) ;

  uint32_t timestamp;
  Serial2.readBytes((char *)&timestamp, sizeof(timestamp));
  #ifdef DEBUG_MAIN
  Serial.print("DEBUG (getTimestampFromGPS) -> Timestamp recibido: ");
  Serial.println(timestamp);
  #endif
  /* Actualizar timestamp. */
}