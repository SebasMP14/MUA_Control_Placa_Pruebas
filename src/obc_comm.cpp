/**
 * @file obc_comm.h
 * Lógica de comunicación con el OBC. Se aplica una FSM (Finite State Machine) de Moore
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - Comunicación con el GPS Orion B16
 * 
 */

#include "obc_comm.h"

OperationMode currentMode = INICIO;
bool setup_state = false;
unsigned long timeOUT = 3000;
unsigned long timeOUT_invalid_frame = 30;
unsigned long timeOUT_window = 1;

uint8_t ack_MUA_to_OBC[TRAMA_COMM] = {0x26, 0x07, 0x00, 0x48, 0x04, 0x0A};            // MUA to OBC ACK
const uint8_t nack_MUA_to_OBC[TRAMA_COMM] = {0x26, 0xFF, 0x00, 0xFF, 0xFF, 0x0A};     // INVALID CHECKSUM NACK
const uint8_t nack_IF_MUA_to_OBC[TRAMA_COMM] = {0x26, 0x00, 0x00, 0x00, 0x00, 0x0A};  // INVALID FRAME RECEIVED NACK

const uint8_t ACK_OBC_to_MUA = 0x04;

RTC_SAMD51 rtc;

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
  uint8_t response[TRAMA_COMM];
  unsigned long tiempo = millis();

  while ( Serial1.available() < TRAMA_COMM ) {            // Esperar respuesta, agregar TimeOut
    if ( tiempo >= timeOUT ) {
      return ;
    }
  }
  delay(100);

  Serial1.readBytes(response, TRAMA_COMM);                //  Se recibe un byte indicando el modo de operación
  #ifdef DEBUG_OBC
  Serial.print("(DEBUG) requestOperationMode -> Recibido de Serial1: 0x");
  for (uint8_t i = 0; i < TRAMA_COMM; i++) {              // trama recibida del OBC
    Serial.print(response[i], HEX);
    Serial.print(", 0x");
  }
  Serial.println();
  #endif

  if ( response[0] != MISSION_ID ) {                       // Comprobación de MISSION ID
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (requestOperationMode) -> ID de Misión incorrecto");
    #endif
    delay(timeOUT_invalid_frame);
    Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);
    return ;
  }
  
  // CRC check
  uint16_t received_CRC = (response[3] << 8) | response[4];
  uint16_t CRC = crc_calculate(response);
  #ifdef DEBUG_OBC
  Serial.print("DEBUG (requestOperationMode) -> CRC calculado: 0x");
  Serial.println(CRC, HEX);
  #endif

  if ( CRC != received_CRC ) {
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (requestOperationMode) -> CRC calculado no coincide con el recibido");
    #endif
    Serial1.write(nack_MUA_to_OBC, TRAMA_COMM);
    return ;
  }

  //  IF INVALID FRAME
  if (  response[1] != ID_COUNT_MODE && 
        response[1] != ID_TRANSFER_MODE && 
        response[1] != ID_TRANSFER_SYSINFO_MODE &&
        response[1] != ID_FINISH ) {
    #ifdef DEBUG_OBC
    Serial.println("DEBUG (requestOperationMode) -> Estado inválido.");
    #endif
    delay(timeOUT_invalid_frame);                     // If an invalid frame is received, a timeout error shall occur
    Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);    // and then a NACK (No-Acknowledgment) message shall be sent
    return ;                                      
  }
  
  // Send ACK if every thing is ok
  CRC = crc_calculate(ack_MUA_to_OBC);
  #ifdef DEBUG_OBC
  Serial.print(" CRC calculado: 0x");
  Serial.println(CRC, HEX);
  #endif
  ack_MUA_to_OBC[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);     // ante penúltima posición
  ack_MUA_to_OBC[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);   // penúltima posición
  Serial1.write(ack_MUA_to_OBC, TRAMA_COMM);

  #ifdef DEBUG_OBC
  Serial.print("DEBUG (requestOperationMode) -> COMAND ID: 0x");
  Serial.println(response[1], HEX);
  #endif
  
  switch (response[1]) {
    case 0x00:
      currentMode = STAND_BY;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> STAND_BY ACTIVATED");
      #endif
      write_OPstate(ID_STANDBY);
      break;
    case 0x01:
      currentMode = COUNT_MODE;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> COUNT MODE ACTIVATED");
      #endif
      write_OPstate(ID_COUNT_MODE);
      break;
    case 0x02:
      currentMode = TRANSFER_DATA_MODE;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> TRANSFER MODE ACTIVATED");
      #endif
      write_OPstate(ID_TRANSFER_MODE);
      break;
    case 0x08:
      currentMode = FINISH;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> FINISH MODE ACTIVATED");
      Serial.println("Sleep mode in progress: Executing order 66.");
      #endif
      write_OPstate(ID_STANDBY);
      enterOffMode();
      break;
    case 0x09:
      currentMode = TRANSFER_INFO_MODE;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> TRANSFER SYSINFO MODE ACTIVATED");
      #endif
      write_OPstate(ID_TRANSFER_SYSINFO_MODE);
      break;
    default:
      currentMode = STAND_BY;
      #ifdef DEBUG_OBC
      Serial.println("DEBUG (requestOperationMode) -> UNKNOWN MODE");
      #endif
      write_OPstate(ID_STANDBY);
      break;
  }

}

/************************************************************************************************************
 * @fn      getTimestampFromGPS
 * @brief   Obtiene el timestamp del GPS y actualiza el reloj del sistema (SIMULACION, AUN FALTA PROGRAMAR)
 * @param   NONE
 * @return  NONE
 *  TODO: - Read GPS
 */
void getTimestampFromGPS(void) {
  Serial2.begin(115200);                // GPS
  #ifdef DEBUG_OBC
  Serial.println("DEBUG (getTimestampFromGPS) -> Serial2 Iniciado");
  #endif

  while ( Serial2.available() ) {
    Serial2.read();
  }
  Serial2.write(0xBB);                  // Comando para solicitar el timestamp
  unsigned long Time = millis();
  while ( Serial2.available() < 4 && millis() - Time < timeOUT) ; // sizeof(uint32_t)

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
 * @fn      slidingWindowBuffer
 * @brief   Obtiene bytes de Serial1 (OBC) hasta detectar una trama válida, se manejan los siguientes tipos de 
 *          errores: timeout, invalid frame and data Noise.
 * @param   buffer: Puntero a memoria donde almacenar trama recibida del OBC
 * @return  true: trama válida recibida ... 
 * @return  false: trama inválida recibida
 */
bool slidingWindowBuffer(uint8_t* buffer) {
  static uint8_t window[6] = {0};         // Sliding window buffer
  // static uint8_t index = 0;
  unsigned long tiempo = millis();

  while ( tiempo - millis() < timeOUT_window ) {
    if ( Serial1.available() ) {
      uint8_t incoming = Serial1.read();    // new byte
      
      for ( uint8_t i = 0; i < 5; i++ ) {   // slide window and add the new byte to the final position
        window[i] = window[i + 1];
      }
      window[5] = incoming;

      // Condición para analizar CRC
      if (window[0] == MISSION_ID &&     // Primer Byte debe ser el ID de MUA MISSION
          // window[2] == 0x00 &&           // Tercer Byte debe ser 0x00 para trama de comunicación
          window[5] == STOP_BYTE) {      // Byte final debe ser 0x0A (STOP)   
        // uint16_t crc_calc = crc_calculate(window);
        // uint16_t crc_recv = (window[3] << 8) | window[4];

        // if ( crc_calc == crc_recv ) {    // Se verifica CRC
          memcpy(buffer, window, 6);     // Trama válida → copiar a buffer
          // return 1;                      // Trama válida encontrada
          return true;
        // } else {
          // Serial1.write(nack_MUA_to_OBC, TRAMA_COMM);
          // window[0] = 0x47;
          // window[5] = 0x47;
          // memcpy(buffer, window, 6);
          // return 2;                      // Trama 
        // }
      }
    }
  }
  
  return false;
}


/************************************************************************************************************
 * @fn      buildDataFrame
 * @brief   Construcción de la trama de datos a enviar al OBC
 * @param   trama: Almacenamiento de datos y comunicacion   
 * @param   ID: Mission ID
 * @param   trama_size: Cantidad de datos a obtener de la memoria flash
 * @param   address: Dirección de inicio de los datos
 * @return  true: Datos cargados con éxito ... 
 * @return  false: Error al cargar datos
 */
bool buildDataFrame(uint8_t* trama, uint8_t ID, uint8_t trama_size, uint32_t address) {
  trama[0] = MISSION_ID;    // MISSION ID
  trama[1] = ID;            // COMMAND ID
  trama[2] = trama_size;    // BYTES QUANTITY TO TRANSFER

  if ( !read(&trama[3], trama_size, address) ) return false; // Cargar datos de flash

  uint16_t CRC = crc_calculate(trama);
  trama[trama_size + 3] = (uint8_t)(CRC >> 8);
  trama[trama_size + 4] = (uint8_t)(CRC & 0xFF);
  trama[trama_size + 5] = 0x0A; // STOP byte

  return true;
}

/************************************************************************************************************
 * @fn      verifyOBCResponse
 * @brief   Verifica el CRC y Mission ID de la trama recibida, en un caso fallido se devuelve un NACK
 * @param   recibido: trama recibida del OBC
 * @return  true: trama recibida verificada correctamente ... 
 * @return  false: trama recibida con errores
 */
bool verifyOBCResponse(uint8_t* recibido) {
  uint16_t crc_expected = crc_calculate(recibido);
  uint16_t crc_received = (recibido[TRAMA_COMM - 3] << 8) | recibido[TRAMA_COMM - 2];

  if ( crc_expected != crc_received ) {
    Serial1.write(nack_MUA_to_OBC, TRAMA_COMM);
    return false;
  }

  if ( recibido[0] != MISSION_ID ) {
    delay(timeOUT_invalid_frame);
    Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);
    return false;
  }

  return true;
}









/**
*******************************************************************************
*                   CRC-16 CCITT TABLE DRIVEN ALGORITHM       
*******************************************************************************
 * using the configuration:
 *  - Width         = 16
 *  - Poly          = 0x1021
 *  - XorIn         = 0x1d0f
 *  - ReflectIn     = False
 *  - XorOut        = 0x0000
 *  - ReflectOut    = False
 *  - Algorithm     = table-driven
*/
const uint16_t crc_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

/**
 * @fn                crc_calculate
 * @brief             Calculate the crc16 value of data[1] to data[-4] (without CRC)
 * @param[in] data    Pointer to a buffer of up to 45 data_len bytes.
 * @return            The calculated crc value.
 */
uint16_t crc_calculate(uint8_t *data) {
  uint16_t crc = 0x1d0f;     //initial crc value
  uint16_t tbl_idx;
  
  // get the amount of appended data according to Data size
  uint8_t data_len = data[2];
  
  // operate only over data[1:data_len] to calculate the checksum 
  for ( uint8_t i = 1; i <= data_len + 2; i++ ) {   
    tbl_idx = (((crc >> 8) ^ (*(data + i))) & 0xFF);
    crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xFFFF;
  }
  
  return (crc & 0xFFFF);
}
// *******************************************************************************
// *******************************************************************************

// /************************************************************************************************************
//  * @fn      calcularCRC
//  * @brief   calcula el Cyclic Redundancy Code - 16 bits, utilizando el polinomio 0x1021
//  * @param   NONE
//  * @return  uint16_t crc
//  */
// uint16_t calcularCRC(const uint8_t *data, size_t length) {
//   uint32_t crc = 0xFFFF;                // Valor inicial
//   uint32_t pol = 0x8408;                // polinomio 
//   uint8_t temp_data[length];
//   uint8_t temp;

//   memcpy(temp_data, data, length);

//   for ( uint8_t i = 0; i < length; i++ ) {
//     for ( uint8_t j = 0; j < 8; j++ ) {
//       temp = (crc ^ temp_data[i]) & 0x0001;
//       crc = crc >> 1;
//       if ( temp == 1 ) {
//         crc = crc ^ pol;
//       }
//       temp_data[i] = temp_data[i] >> 1;
//     }
//   }
  
//   return crc ^ 0xFFFF;
// }


/************************************************************************************************************
 * @fn      requestOperationMode (CRC en cada trama, con todas las tramas con longitud TRAMA_SIZE)
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

* * **********************************************************************************************************
 * @fn      requestOperationMode (En esta version, la AEP había solicitado un solo byte de comunicación)
 * @brief   Espera el modo de operación de la misión, establecida por el OBC
 * @param   NONE
 * @return  NONE
 * TODO:
 * - Cambiar los baudios en los protocolos UART a 9600
 * - Agregar Timeout
 * - Agregar estado TRANSFER_INFO_MODE
 *
void requestOperationMode(void) {
  uint8_t response;
  
  while (Serial1.available() != 1) ;               // Esperar respuesta, agregar TimeOut
  delay(100);

  Serial1.readBytes(&response, 1);                //  Se recibe un byte indicando el modo de operación
  //  Si la respuesta no corresponde a ningún estado de operación
  if ( response != ID_COUNT_MODE && response != ID_TRANSFER_MODE && response != ID_TRANSFER_SYSINFO_MODE ) {   
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
    case 0x09:
      currentMode = TRANSFER_INFO_MODE;
      #ifdef DEBUG_OBC
      Serial.println("TRANSFER SYSINFO MODE ACTIVATED");
      #endif
      write_OPstate(0x09);
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



*/