/**
 * max1932_driver.cpp
 * Funciones de control del max1932, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Command de la función write debería ser la tensión de salida deseada
 */
#include "max1932_driver.h"
#include <Arduino.h>

/************************************************************************************************************
 * @fn      start_max1932
 * @brief   Inicializa el puerto SPI
 * @param   NONE
 * @return  NONE
 */
void start_max1932(void) {
  SPI.begin();  
}

/************************************************************************************************************
 * @fn      write_max_reg
 * @brief   Envía el comando de tensión
 * @param   command: valores de cero a 255, siendo 255 la instrucción que genera la menor tensión de salida
 *                   en el max1932 y 1 el mayor valor, 0 apaga el convertidor
 * @return  true exitoso - false fallido
 */
bool write_max_reg(uint8_t command) {
  #ifdef DEBUG_MAX
  Serial.print("0x");
  Serial.println(command, HEX);
  #endif

  SPI.beginTransaction(SPISettings(SPI_CLK_Speed, MSBFIRST, SPI_MODE0)); // 2 MHz máximo
  digitalWrite(SPI_CS_MAX, LOW);  // selección
  SPI.transfer(command);          // Envio de comando
  SPI.endTransaction();
  delayMicroseconds(2);
  digitalWrite(SPI_CS_MAX, HIGH);

  return true;
}