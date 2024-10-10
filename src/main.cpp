#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"

#define P PA18 // Blink

uint8_t status = 0;
float temperature = 0.0;

void setup() {
  delay(4000);

  uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};  /* Datos de ejemplo de escritura */
  bool flash_status = true;

  Serial.begin(115200);
  Serial.println("Iniciado");

  // // Configuración del TMP100
  // if ( !start_tmp100() ) {
  //   #ifdef DEBUG_MODE
  //   Serial.println("Inicialización de TMP100 fallida");
  //   #endif
  // }

  pinMode(PULSE_1, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  pinMode(SCL_Sensor, OUTPUT); // salida para Timer Control 2 
  
  digitalWrite(P, LOW);

  attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1, RISING);

  flash_status = start_flash();
  if ( flash_status == true ) {
    #ifdef DEBUG_MODE
    Serial.println("Comunicar al OBC que el flash no conecta.");
    #endif
    flash_status = true;
  }

  flash_status = write_mem((uint8_t *)&data, sizeof(data));
  if ( flash_status == true ) {
    #ifdef DEBUG_MODE
    Serial.println("Fallo en la escritura.");
    #endif
    flash_status = true;
  }

  read_all();

  Serial.println("Setup finalizado...");

}

void loop() {

  digitalWrite(P, HIGH);
  delay(1000);
  digitalWrite(P, LOW);

  if (detect == true) {
    #ifdef DEBUG_MAIN
    Serial.println(pulse_count1);
    #endif
    detect = false;
  }

  // temperature = read_tmp100();
  // if (isnan(temperature)) {
  //   #ifdef DEBUG_MODE
  //   Serial.println("Error al leer la temperatura.");
  //   #endif
  // } else {
  //   Serial.print("TMP: ");
  //   Serial.print(temperature);
  //   Serial.println(" ºC");
  // }
  
}
