/**
 * @file flash_driver.cpp
 * Funciones de control de la memoria flash (MT25QL01GBBB) para la misión M.U.A. de FIUNA 
 *  La memoria flash se separa en dos secciones de almacenamiento:
 * - La sección de DATOS almacena las tramas generadas en loopCount y va desde el sector cero hasta 
 * el penúltimo sector de la memoria flash
 * - La sección SYSINFO almacena variables importantes de operación del sistema de control y abarca sólo
 * el último sector de la memoria flash
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github: https://github.com/SebasMP14)
 * - Ing. Lucas Cho 
 * 
 * TODO:
 * - 
 */

#include "flash_driver.h"

Adafruit_FlashTransport_QSPI flashTransport(QSPI_SCK, QSPI_CS, QSPI_D0, QSPI_D1, QSPI_D2, QSPI_D3);
Adafruit_SPIFlash Flash_QSPI(&flashTransport);

/************************************************************************************************************
 * @fn      read
 * @brief   Se obtiene en el puntero data los bytes desde la dirección since
 * @param   data: Puntero de datos
 * @param   data_length: cantidad de datos
 * @param   since: dirección desde dónde se lee la memoria
 * @return  True - False
 */
bool read(uint8_t *data, uint32_t data_length, uint32_t since) {
  uint32_t len_bytes;
  len_bytes = Flash_QSPI.readBuffer(since, data, data_length);
  if ( len_bytes == 0 ) {                      /* Si existe un error en la lectura */ 
    #ifdef DEBUG_FLASH
    Serial.println("ERROR (read_OPstate) -> Error en la lectura de la memoria FLASH.");
    #endif
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (read_SENDED_DATAaddress) -> Dirección en memoria: 0x");
  Serial.print(LAST_ADDRESS_SENT_DIR, HEX);
  Serial.print(" Ultima dirección enviada: 0x");
  Serial.println(*data, HEX);
  #endif
  return true;
}

/************************************************************************************************************
 * @fn      get_SENDED_DATAaddress
 * @brief   Escribe la dirección del último dato enviado al obc de la sección de datos
 * @param   address: Dirección a ser almacenada
 * @return  True - False
 */
bool get_SENT_DATAaddress(uint32_t *sent_address) {
  uint32_t len_bytes;
  len_bytes = Flash_QSPI.readBuffer(LAST_ADDRESS_SENT_DIR, reinterpret_cast<uint8_t*>(sent_address), ADDRESS_SIZE);
  if ( len_bytes == 0 ) {                      /* Si existe un error en la lectura */ 
    #ifdef DEBUG_FLASH
    Serial.println("ERROR (read_OPstate) -> Error en la lectura de la memoria FLASH.");
    #endif
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (read_SENDED_DATAaddress) -> Dirección en memoria: 0x");
  Serial.print(LAST_ADDRESS_SENT_DIR, HEX);
  Serial.print(" Ultima dirección enviada: 0x");
  Serial.println(*sent_address, HEX);
  #endif
  return true;
}

/************************************************************************************************************
 * @fn      write_SENDED_DATAaddress
 * @brief   Escribe la dirección del último dato enviado al obc de la sección de datos
 * @param   address: Dirección a ser almacenada
 * @return  True - False
 */
bool write_SENT_DATAaddress(uint8_t address) {
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (write_SENDED_DATAaddress) -> ...");
  #endif
  return write_DATAinfo(&address, ADDRESS_SIZE, SENT_INDEX);
}

/************************************************************************************************************
 * @fn      write_DATAinfo
 * @brief   Escribe en el espacio SYSINFO de la memoria (último sector de la flash)
 * @param   buffer: Puntero de datos a escribir
 * @param   len: Cantidad de bytes a escribir
 * @param   index: Desde dónde se escribirá
 * @return  true: exito - false: fallo
 */
bool write_DATAinfo(uint8_t *buffer, uint32_t len, uint16_t index) {
  if ( index + len > SIZE_INFO ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_DATAinfo) -> Datos exceden el tamaño permitido.");
    #endif
    return false;
  }

  uint8_t saved[SIZE_INFO];

  // Leer los datos actuales del sector
  if ( !Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, saved, SIZE_INFO) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_DATAinfo) -> Error al leer los datos existentes.");
    #endif
    return false;
  }

  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (write_DATAinfo) -> Datos existentes antes de la actualización: ");
  for ( uint8_t i = 0; i < SIZE_INFO; i++ ) {
    Serial.print("0x");
    Serial.print(saved[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  #endif

  // Actualizar los datos en memoria temporal
  memcpy(&saved[index], buffer, len);         // Se escribe desde index hasta index + len

  // Borrar el sector
  if ( !Flash_QSPI.eraseSector(SAVED_SYSINFO_SECTOR) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_DATAinfo) -> Falló el borrado del sector.");
    #endif
    return false;
  }

  // Escribir los datos actualizados
  uint32_t len_bytes = Flash_QSPI.writeBuffer(SAVED_ADDRESS_SECTOR_DIR, saved, SIZE_INFO);
  if ( len_bytes == 0 ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_DATAinfo) -> Error al escribir los datos actualizados.");
    #endif
    return false;
  }

  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (write_DATAinfo) -> Datos actualizados correctamente:");
  for ( uint8_t i = 0; i < SIZE_INFO; i++ ) {
    Serial.print("0x");
    Serial.print(saved[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  #endif

  return true;
}

/************************************************************************************************************
 * @fn      read_OPstate
 * @brief   Lee el espacio de memoria donde se guarda el último estado de operación del sistema.
 * @param   NONE
 * @return  NONE
 */
bool get_OPstate(uint8_t *read) {
  uint32_t len_bytes;
  len_bytes = Flash_QSPI.readBuffer(ADDRESS_OP_STATE_DIR, read, 1);
  if ( len_bytes == 0 ) {                      /* Si existe un error en la lectura */ 
    #ifdef DEBUG_FLASH
    Serial.println("ERROR (read_OPstate) -> Error en la lectura de la memoria FLASH.");
    #endif
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (read_OPstate) -> Dirección: 0x");
  Serial.print(ADDRESS_OP_STATE_DIR, HEX);
  Serial.print(" Estado leído: 0x");
  Serial.println(*read, HEX);
  #endif
  return true;
}

/************************************************************************************************************
 * @fn      write_OPstate
 * @brief   escribe el último estado de operación del sistema.
 * @param   NONE
 * @return  NONE
 */
bool write_OPstate(uint8_t state) {
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (write_OPstate) -> ");
  #endif
  return write_DATAinfo(&state, 1, 11); // Índice 11 para el estado de operación
}

/************************************************************************************************************
 * @fn      read_all
 * @brief   lee todos los datos de la memoria
 * @param   NONE
 * @return  NONE
 */
bool erase_all(void) {
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (erase_all) -> Borrando...");
  #endif
  return Flash_QSPI.eraseChip();
}

/************************************************************************************************************
 * @fn      read_all
 * @brief   lee todos los datos de la memoria
 * @param   NONE
 * @return  NONE
 */
void read_all(void) {
  uint32_t new_write_address = 0; // Posición actual de escritura
  uint8_t read_byte = 0;          // Almacenamiento de datos leídos

  if ( !get_address(&new_write_address) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (read_all) -> Error al obtener Dir.");
    #endif
    return ;
  }

  #ifdef DEBUG_FLASH_INFO
  Serial.println("DEBUG (read_all) -> Leyendo los datos escritos: ");
  #endif
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (read_all) -> ");
  #endif
  for (uint32_t address = 0x00; address < new_write_address; address++) {
    if ( Flash_QSPI.readBuffer(address, &read_byte, 1) ) {
      #ifdef DEBUG_FLASH
      Serial.print("0x");
      Serial.print(read_byte, HEX);
      Serial.print(" ");
      #endif
    } else {
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (read_all) -> Error al leer de la memoria.");
      #endif
    }
  }
  #ifdef DEBUG_FLASH
  Serial.println();
  #endif
}

/************************************************************************************************************
 * @fn      read_until
 * @brief   Lee una cantidad específica de Bytes
 * @param   NONE
 * @return  NONE
 * TODO: - Analizar si es necesario...
 *  - ver si data_length es x4
 */
void read_until(uint8_t *data, uint32_t data_length) {
  uint32_t write_address = 0; // Posición actual de escritura
  uint8_t read_byte = 0;          // Almacenamiento de datos leídos

  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (read_until) -> ");
  #endif

  get_address(&write_address);
  if ( data_length < write_address ) {
    if ( Flash_QSPI.readBuffer(0x00, data, data_length) ) {
      #ifdef DEBUG_FLASH_INFO
      Serial.print("0x");
      Serial.print(read_byte, HEX);
      Serial.print(" ");
      // Serial1.write(read_byte); // Transferencia de byte al OBC
      #endif
      return ;
    } else {
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (read_until) -> Error al leer de la memoria.");
      #endif
    }
  } else {
    if (Flash_QSPI.readBuffer(0x00, data, write_address)) { // leer hasta la ultima dir de mem escrita
      #ifdef DEBUG_FLASH_INFO
      Serial.print("0x");
      Serial.print(read_byte, HEX);
      Serial.print(" ");
      // Serial1.write(read_byte); // Transferencia de byte al OBC
      #endif
      return ;
    } else {
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (read_until) -> Error al leer de la memoria.");
      #endif
    }
  }
  Serial.println();
}

/************************************************************************************************************
 * @fn      write_mem
 * @brief   Escribe los datos en la primera posición disponible. Se guarda de LSB a MSB (little-endian).
 * @param   buffer: Buffer con los datos a escribir
 * @param   len:    Longitud total de estos
 * @return  true exitoso - false fallido
 */
bool write_mem(uint8_t *buffer, uint32_t len) {
  uint32_t len_bytes = 0;
  uint32_t new_write_address = 0;

  get_address(&new_write_address);

  /////// Escritura de datos
  len_bytes = Flash_QSPI.writeBuffer(new_write_address, (uint8_t *)buffer, len); 
  if ( len_bytes == 0 ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_mem) -> Error al escribir en la memoria.");
    #endif
    return false;
  }

  new_write_address += len_bytes;
  update_address(&new_write_address);
  
  return true;
}

/************************************************************************************************************
 * @fn      update_address
 * @brief   Se actualiza la siguiente dirección disponible para escritura en la memoria (al inicio del último
 * sector)
 * @param   address: Dirección a alamacenar
 * @return  true exitoso - false fallido
 */
bool update_address(uint32_t *address) {
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (update_address) -> ...");
  #endif
  return write_DATAinfo((uint8_t *)address, ADDRESS_SIZE, 0);
}

/************************************************************************************************************
 * @fn      get_address 
 * @brief   Retorna la ultima direccion donde existen datos
 * @param   write_address: Variable donde sera almacenada la direccion resultante
 * @return  true sin error - false error en lectura
 */
bool get_address( uint32_t *write_address ) {
  uint32_t len_bytes = 0;                     /* Bytes escritos/leidos por/de la memoria FLASH */

  /**   Se leen 4 bytes empezando en la dirección del sector 
   *  reservado para almacenar la ultima dirección de memoria 
   */ 
  len_bytes = Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t*)write_address, ADDRESS_SIZE);   
  if ( len_bytes == 0 ) {                      /* Si existe un error en la lectura */ 
    #ifdef DEBUG_FLASH
    Serial.println("ERROR (get_address) -> Error en la lectura de la memoria FLASH.");
    #endif
    *write_address = 0x00000000;
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (get_address) -> Ultima dirección: 0x");
  Serial.println(*write_address, HEX);
  #endif
  
  if (*write_address == 0xFFFFFFFF) {   // Si la última direccion es NULL, se inicia 
    *write_address = 0x00000000;        // desde la posición 0 de la memoria FLASH 
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (get_address) -> Iniciando escritura en la direccion 0.");
    #endif
  }

  return true;      /* Retornar si no ocurren errores */
}



/************************************************************************************************************
 * @fn      start_flash
 * @brief   Inicia la memoria Flash
 * @param   NONE
 * @return  true sin error
 *          false error al iniciar FLASH
 */
bool start_flash(void) {
  flashTransport.begin(); // Inicializa QSPI solo en el MCU (SAMD51)
  #ifdef DEBUG_FLASH_INFO
  if (flashTransport.supportQuadMode()) {
    Serial.println("DEBUG (start_flash) -> La memoria Flash soporta QSPI.");
  } else {
    Serial.println("DEBUG (start_flash) -> La memoria Flash no soporta QSPI.");
  }
  #endif

  /* Intentar iniciar FLASH QSPI */
  for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITERATIONS ; iter_counter ++) {
    if ( Flash_QSPI.begin() ) {               // Verificar si el FLASH QSPI ya inicio
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (start_flash) -> Memoria flash iniciada correctamente.");
      #ifdef DEBUG_FLASH_INFO
      if (Flash_QSPI.isReady()) {
        Serial.println("DEBUG (start_flash) -> La mem flash Está lista");
        Serial.print("DEBUG (start_flash) -> ID JEDEC de la memoria flash: 0x");
        Serial.println(Flash_QSPI.getJEDECID(), HEX);  
      }
      #endif
      Serial.println("DEBUG (start_flash) -> Memoria flash_QSPI inicializada correctamente.");
      #endif
      Flash_QSPI.setIndicator(LED_BUILTIN, true);
      return true;
    }
    #ifdef DEBUG_FLASH_INFO
    Serial.println("ERROR (start_flash) -> Error al inicializar la memoria Flash.");
    #endif
    delay(10);
  }

  /* Si no se pudo iniciar en el numero maximo de intentos */
  #ifdef DEBUG_FLASH
  Serial.println("ERROR (start_flash) -> No se pudo iniciar la memoria Flash.");
  #endif

  return false;
}

/************************************************************************************************************
 * @fn      write_OPstate1
 * @brief   escribe el último estado de operación del sistema.
 * @param   NONE
 * @return  NONE
 *
bool write_OPstate1(uint8_t state) {
  uint8_t saved[SIZE_INFO];
  if ( Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, saved, SIZE_INFO) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (write_OPstate) -> Datos leídos: ");
    for (uint8_t i = 0; i < SIZE_INFO; i++) {
      Serial.print("0x");
      Serial.print(saved[i], HEX);
      Serial.print(" ");
    }
    #endif
  } else {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (read_all) -> Error al leer de la memoria.");
    #endif
  }

  saved[11] = state;

  if ( !Flash_QSPI.eraseSector(SAVED_SYSINFO_SECTOR) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (update_address) -> Borrado de sector fallido.");
    #endif
    return false;
  }

  uint8_t len_bytes = Flash_QSPI.writeBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t*)saved, SIZE_INFO);
  if (len_bytes == 0) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (update_address) -> Error al guardar dirección en la memoria.");
    #endif
    // TODO: retornar error en una funcion 
    return false;
  }

  #ifdef DEBUG_FLASH
  if ( Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, saved, 1) ) {
    Serial.println("DEBUG (write_OPstate) -> Datos actualizados: ");
    for (uint8_t i = 0; i < SIZE_INFO; i++) {
      Serial.print("0x");
      Serial.print(saved[i], HEX);
      Serial.print(" ");
    }
  } else {
    Serial.println("DEBUG (read_all) -> Error al leer de la memoria.");
  }
  #endif

  return true;
}*/

/************************************************************************************************************
 * @fn      update_address1
 * @brief   Se actualiza la siguiente dirección disponible para escritura en la memoria (al inicio del último
 * sector)
 * @param   address: Dirección a alamacenar
 * @return  true exitoso - false fallido
 *
bool update_address1(uint32_t *address) {
  uint32_t len_bytes = 0;
  uint32_t read_data = 0;

  **  Se borra el sector donde es almacenada la ultima direccion de escritura
   * esto se debe a que no se permite la sobreescritura de datos, y solamente esta habilitado
   * borrar datos por sectores, no por páginas.
   *
  if ( !Flash_QSPI.eraseSector(SAVED_SYSINFO_SECTOR) ) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (update_address) -> Borrado de sector fallido.");
    #endif
    return false;
  }

  // Se actualiza la dirección con la cantidad de bytes escritos 
  len_bytes = Flash_QSPI.writeBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t*)address, ADDRESS_SIZE);
  if (len_bytes == 0) {
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (update_address) -> Error al guardar dirección en la memoria.");
    #endif
    // TODO: retornar error en una funcion 
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (update_address) -> Dirección a almacenar: 0x");
  Serial.println(*address, HEX);
  #endif
  
  len_bytes = Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t *)&read_data, ADDRESS_SIZE);
  if (len_bytes == 0) {
    Serial.print("DEBUG (update_address) -> Error leyendo de la memoria FLASH");
    // TODO: retornar error en una funcion 
    return false;
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (update_address) -> Dirección almacenada: 0x");
  Serial.println(read_data, HEX);
  #endif

  return true;
}*/
