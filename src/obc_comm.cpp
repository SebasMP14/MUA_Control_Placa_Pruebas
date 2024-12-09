#include "obc_comm.h"

OperationMode currentMode = UNKNOWN_MODE;
bool setup_state = false;
RTC_SAMD51 rtc;

/************************************************************************************************************
 * @fn      requestOperationMode
 * @brief   Solicita al OBC el modo de operación de la misión.
 * @param   NONE
 * @return  NONE
 * TODO:
 * - Cambiar los baudios en los protocolos UART a 9600
 */
void requestOperationMode(void) {
  Serial1.begin(115200);                // OBC
  #ifdef DEBUG_OBC
  Serial.println("DEBUG (requestOperationMode) -> Serial1 Iniciado");
  #endif

  Serial1.write(0xAA);                  // Comando para solicitar el modo
  
  while (Serial1.available() < 0) ;
  delay(100);
  uint8_t response;
  Serial1.readBytes(&response, 1);      // TRAMA del OBC

  #ifdef DEBUG_OBC
  Serial.print("DEBUG (requestOperationMode) -> Recibido de Serial1 0x");
  Serial.println(response, HEX);
  #endif
  
  if (response == 0xA1) {               // Decodificar la respuesta y establecer el modo
    currentMode = COUNT_MODE;
    #ifdef DEBUG_OBC
    Serial.println("COUNT MODE ACTIVATED");
    #endif
  } else if (response == 0xA4) {
    currentMode = TRANSFER_MODE;
    #ifdef DEBUG_OBC
    Serial.println("TRANSFER MODE ACTIVATED");
    #endif
  } else {
    currentMode = UNKNOWN_MODE;
    #ifdef DEBUG_OBC
    Serial.println("UNKNOWN MODE ACTIVATED");
    #endif
  }
}

/************************************************************************************************************
 * @fn      getTimestampFromGPS
 * @brief   Obtiene el timestamp del GPS y actualiza el reloj del sistema con este.
 * @param   NONE
 * @return  NONE
 */
void getTimestampFromGPS(void) {
  Serial2.begin(115200);                // GPS
  #ifdef DEBUG_OBC
  Serial.println("DEBUG (getTimestampFromGPS) -> Serial2 Iniciado");
  #endif

  Serial2.write(0xBB);  // Comando para solicitar el timestamp
  
  while ( Serial2.available() < sizeof(uint32_t) ) ;

  unsigned long timestamp;
  Serial2.readBytes((char *)&timestamp, sizeof(timestamp));

  #ifdef DEBUG_OBC
  Serial.print("DEBUG (getTimestampFromGPS) -> Timestamp recibido: ");
  Serial.println(timestamp);
  #endif
  
  /* Actualizar timestamp del sistema. */
  DateTime dT = DateTime(timestamp);
  rtc.adjust(dT);
}

unsigned long getTime(void) { 
  return rtc.now().unixtime();
}