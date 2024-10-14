#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
// #include "max1932_driver.h"

#define DEBUG_MAIN
#define P PB09 // Blink
#define PULSE PB08

// uint8_t status = 0;
float temperature = 0.0;
union FloatToUint32 { // para evitar aliasing y no violar las reglas del compilador
  float f;
  uint32_t u;
};
unsigned long time_ini;

void setup() {
  delay(4000);

  Serial.begin(115200);
  Serial.println("Iniciado");

  pinMode(PULSE, INPUT_PULLDOWN);
  // pinMode(PULSE_2, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  // pinMode(SCL_Sensor, OUTPUT); // salida para Timer Control 2 
  pinMode(PA01, OUTPUT); // Salida para TC2 (utiliza también PA15)
  
  digitalWrite(P, LOW);

  attachInterrupt(digitalPinToInterrupt(PULSE), handlePulse1, RISING);
  // attachInterrupt(digitalPinToInterrupt(PULSE), handlePulse1_FALLING, FALLING);
  setupTC2();
  setupTC_Pulse1();

  // Configuración del TMP100
  if ( !start_tmp100() ) {
    #ifdef DEBUG_MAIN
    Serial.println("Inicialización de TMP100 fallida");
    #endif
  }

  if ( !start_flash() ) {
    #ifdef DEBUG_MAIN
    Serial.println("Comunicar al OBC que el flash no conecta.");
    #endif
  }

  // erase_all();

  time_ini = millis();

  Serial.println("Setup finalizado...");
}

void loop() {

  digitalWrite(P, HIGH);
  delay(100);
  digitalWrite(P, LOW);

  if (detect1) {
    #ifdef DEBUG_MAIN
    Serial.print("Pulsos: ");
    Serial.print(pulse_count1);
    Serial.print(", ");
    // Serial.println(pulse_count2);
    Serial.print("Duración: ");
    Serial.print(pulse_duration);
    Serial.println(" ticks");
    #endif
    detect1 = false;
    // detect2 = false;
  }

  if ( detect_TC ) {
    read_all();
    detect_TC = false;
  }

  if ( (millis() - time_ini) >= 5000 ) {
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
    time_ini = millis();
  }


  
}
