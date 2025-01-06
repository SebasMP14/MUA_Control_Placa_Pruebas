/**
 * obc_comm.h
 * Lógica de comunicación con el OBC. Se aplica una FSM (Finite State Machine) de Moore
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Comunicación con el GPS Orion B16
 * 
 */

#include "obc_comm.h"

OperationMode currentMode = INICIO;
bool setup_state = false;
RTC_SAMD51 rtc;
uint8_t ack_MUA_to_OBC = 0x07; // De esta forma, Los demás valores se inicializan en cero
unsigned long timeOUT = 3000;

/************************************************************************************************************
 * @fn      requestOperationMode
 * @brief   Espera el modo de operación de la misión, establecida por el OBC
 * @param   NONE
 * @return  NONE
 * TODO:
 * - Cambiar los baudios en los protocolos UART a 9600
 * - Agregar Timeout
 * - Agregar estado TRANSFER_INFO_MODE
 */
void requestOperationMode(void) {
  uint8_t response;
  
  while (Serial1.available() != 1) ;               // Esperar respuesta, agregar TimeOut
  delay(100);

  Serial1.readBytes(&response, 1);                //  Se recibe un byte indicando el modo de operación
  if ( response != ID_COUNT_MODE && response != ID_TRANSFER_MODE ) {   //  Si la respuesta no corresponde a ningún estado de operación
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (requestOperationMode) -> Estado inválido.");
    #endif
    return ;                                      //  No se envía ACK
  }
  if ( response == ID_TRANSFER_MODE ) {
    currentMode = TRANSFER_DATA_MODE;
    #ifdef DEBUG_OBC
    Serial.println("TRANSFER MODE ACTIVATED");
    #endif
    return ;
  }
  Serial1.write(ACK_MUA_TO_OBC);

  #ifdef DEBUG_OBC
  Serial.print("DEBUG (requestOperationMode) -> Recibido de Serial1: 0x");
  Serial.println(response, HEX);
  #endif
  
  switch (response) {
    case 0x00:
      currentMode = STAND_BY;
      #ifdef DEBUG_OBC
      Serial.println("STAND_BY ACTIVATED");
      #endif
      write_OPstate(0x00);
      break;
    case 0x01:
      currentMode = COUNT_MODE;
      #ifdef DEBUG_OBC
      Serial.println("COUNT MODE ACTIVATED");
      #endif
      write_OPstate(0x01);
      break;
    case 0x02:
      currentMode = TRANSFER_DATA_MODE;
      #ifdef DEBUG_OBC
      Serial.println("TRANSFER MODE ACTIVATED");
      #endif
      write_OPstate(0x02);
      break;
    default:
      currentMode = STAND_BY;
      #ifdef DEBUG_OBC
      Serial.println("UNKNOWN MODE ACTIVATED");
      #endif
      break;
  }

  // 
}

/************************************************************************************************************
 * @fn      getTimestampFromGPS
 * @brief   Obtiene el timestamp del GPS y actualiza el reloj del sistema
 * @param   NONE
 * @return  NONE
 */
void getTimestampFromGPS(void) {
  Serial2.begin(115200);                // GPS
  #ifdef DEBUG_OBC
  Serial.println("DEBUG (getTimestampFromGPS) -> Serial2 Iniciado");
  #endif

  Serial2.write(0xBB);  // Comando para solicitar el timestamp
  unsigned long Time = millis();
  while ( Serial2.available() < sizeof(uint32_t) && millis() - Time < timeOUT) ;

  if ( millis() - Time > timeOUT ) {
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (getTimestampFromGPS) -> Time Out");
    #endif
    return ;
  }

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

/************************************************************************************************************
 * @fn      getTime
 * @brief   Obtiene la marca temporal del sistema
 * @param   NONE
 * @return  Unixtime
 */
unsigned long getTime(void) {
  return rtc.now().unixtime();
}

/************************************************************************************************************
 * @fn      calcularCRC
 * @brief   calcula el Cyclic Redundancy Code - 16 bits, utilizando el polinomio 0x1021
 * @param   NONE
 * @return  uint16_t crc
 */
uint16_t calcularCRC(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFF;                // Valor inicial
  uint32_t pol = 0x8408;                // polinomio 
  uint8_t temp_data[length];
  uint8_t temp;

  memcpy(temp_data, data, length);

  for ( uint8_t i = 0; i < length; i++ ) {
    for ( uint8_t j = 0; j < 8; j++ ) {
      temp = (crc ^ temp_data[i]) & 0x0001;
      crc = crc >> 1;
      if ( temp == 1 ) {
        crc = crc ^ pol;
      }
      temp_data[i] = temp_data[i] >> 1;
    }
  }
  
  return crc ^ 0xFFFF;
}


/************************************************************************************************************
 * @fn      requestOperationMode (CRC en cada trama)
 * @brief   Espera el modo de operación de la misión establecida por el OBC
 * @param   NONE
 * @return  NONE
 * TODO:
 * - Cambiar los baudios en los protocolos UART a 9600
 *
void requestOperationMode(void) {
  uint8_t response[TRAMA_SIZE];
  while (Serial1.available() < TRAMA_SIZE) ;
  delay(100);
  Serial1.readBytes(response, TRAMA_SIZE);      // TRAMA del OBC

  uint16_t CRC_OBC = (response[TRAMA_SIZE-2] << 8) | response[TRAMA_SIZE-1];
  uint16_t CRC = calcularCRC(response, TRAMA_SIZE-2);
  if ( CRC != CRC_OBC ) {
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (requestOperationMode) -> El CRC no coincide");
    #endif
    return ;
  }

  uint16_t CRC_ACK = calcularCRC(ACK_MUA_TO_OBC, TRAMA_SIZE-2);
  ACK_MUA_TO_OBC[TRAMA_SIZE-2] = (CRC_ACK >> 8) & 0xFF; // CRCH
  ACK_MUA_TO_OBC[TRAMA_SIZE-1] = CRC_ACK & 0xFF;        // CRCL
  Serial1.write(ACK_MUA_TO_OBC, TRAMA_SIZE);

  #ifdef DEBUG_OBC
  Serial.print("DEBUG (requestOperationMode) -> Recibido de Serial1 ");
  for (int i = 0; i < TRAMA_SIZE; i++) {
    Serial.print("0x");
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  #endif
  
  switch (response[0]) {
    case 0x00:
      currentMode = STAND_BY;
      #ifdef DEBUG_OBC
      Serial.println("STAND_BY ACTIVATED");
      #endif
      break;
    case 0x01:
      currentMode = COUNT_MODE;
      #ifdef DEBUG_OBC
      Serial.println("COUNT MODE ACTIVATED");
      #endif
      break;
    case 0x02:
      currentMode = TRANSFER_MODE;
      #ifdef DEBUG_OBC
      Serial.println("TRANSFER MODE ACTIVATED");
      #endif
      break;
    default:
      currentMode = UNKNOWN_MODE;
      #ifdef DEBUG_OBC
      Serial.println("UNKNOWN MODE ACTIVATED");
      #endif
      break;
  }

  // 
}
*/