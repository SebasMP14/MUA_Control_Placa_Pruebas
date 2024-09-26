#include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"

#define P PA18 // Blink

void setup() {
  delay(4000);

  Serial.begin(115200);
  Serial.println("Iniciado");

  pinMode(PULSE_1, INPUT_PULLDOWN);
  pinMode(P, OUTPUT);
  pinMode(SCL_Sensor, OUTPUT); // salida para Timer Control 2 
  
  digitalWrite(P, LOW);

  attachInterrupt(digitalPinToInterrupt(PULSE_1), handlePulse1, RISING);

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
  
}
