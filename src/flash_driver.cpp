/**
 * flash_driver.cpp
 * Funciones de control de la memoria flash (W25Q128JV_SM) para la misión M.U.A. de FIUNA 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * - Ing. Lucas Cho 
 * 
 * 
 * 
 * TODO:
 * - Escribir mensajes al OBC
 * - read_all() debe transferir los datos por UART y luego ejecutar erase_all()
 * - En update_address() intentar escribir varias veces si falla
 */

#include "flash_driver.h"

Adafruit_FlashTransport_QSPI flashTransport(QSPI_SCK, QSPI_CS, QSPI_D0, QSPI_D1, QSPI_D2, QSPI_D3);
Adafruit_SPIFlash Flash_QSPI(&flashTransport);

/************************************************************************************************************
 * @fn      read_all
 * @brief   lee todos los datos de la memoria
 * @param   NONE
 * @return  NONE
 */
bool erase_all() {
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

  get_address(&new_write_address);

  #ifdef DEBUG_FLASH_INFO
  Serial.println("DEBUG (read_all): Leyendo los datos escritos: ");
  #endif
  #ifdef DEBUG_FLASH
  Serial.println("DEBUG (read_all): ");
  #endif
  for (uint32_t address = 0x00; address < new_write_address; address++) {
    if (Flash_QSPI.readBuffer(address, &read_byte, 1)) {
      #ifdef DEBUG_FLASH
      Serial.print("0x");
      Serial.print(read_byte, HEX);
      Serial.print(" ");
      #endif
    } else {
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (read_all): Error al leer de la memoria.");
      #endif
    }
  }
  Serial.println();
}

/************************************************************************************************************
 * @fn      write_mem
 * @brief   escribe los datos en la primera posición disponible
 * @param   buffer, len: buffer con los datos a escribir y la longitud total de estos
 * @return  true exitoso - false fallido
 */
bool write_mem(uint8_t *buffer, uint32_t len) {
  uint32_t len_bytes = 0;
  uint32_t new_write_address = 0;

  get_address(&new_write_address);

  /////// Escritura de datos
  len_bytes = Flash_QSPI.writeBuffer(new_write_address, (uint8_t *)buffer, len); 
  if ( len_bytes == 0 ) {
    Serial.println("Error al escribir en la memoria.");
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
  uint32_t len_bytes = 0;
  uint32_t read_data = 0;
  /*
   * Se borra el sector donde es almacenada la ultima direccion de escritura
   * esto se debe a que no se permite la sobreescritura de datos, y solamente esta habilitado
   * borrar datos por sectores
   */
  Flash_QSPI.eraseSector(SAVED_ADDRESS_SECTOR);

  /* Se actualiza la dirección con la cantidad de bytes escritos */
  len_bytes = Flash_QSPI.writeBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t*)address, ADDRESS_SIZE);
  if (len_bytes == 0) {
    Serial.println("Error al guardar direccion en la memoria.");
    /* TODO: retornar error en una funcion */
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (write_mem): Dirección a almacenar: 0x");
  Serial.println(*address, HEX);
  #endif
  
  len_bytes = Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t *)&read_data, ADDRESS_SIZE);
  if (len_bytes == 0) {
    Serial.print("Error leyendo de la memoria FLASH");
    /* TODO: retornar error en una funcion */
  }
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (write_mem): Dirección almacenada: 0x");
  Serial.println(read_data, HEX);
  #endif

  return true;
}

/************************************************************************************************************
 * @fn      get_address 
 * @brief   Retorna la ultima direccion donde existen datos
 * @param   write_address: Variable donde sera almacenada la direccion resultante
 * @return  true sin error - false error en lectura
 */
bool get_address( uint32_t *write_address ) {
  uint32_t len_bytes = 0;                     /* Bytes escritos/leidos por/de la memoria FLASH */

  /* Se leen 4 bytes empezando en la direccion del sector reservado para almacenar la ultima direccion de memoria */  
  len_bytes = Flash_QSPI.readBuffer(SAVED_ADDRESS_SECTOR_DIR, (uint8_t*)write_address, ADDRESS_SIZE);
  #ifdef DEBUG_FLASH
  Serial.print("DEBUG (get_address): Ultima direccion: 0x");
  Serial.println(*write_address, HEX);
  #endif
  /* Si existe un error en la lectura */  
  if (len_bytes == 0 ) {
    #ifdef DEBUG_FLASH
    Serial.println("ERROR (get_address): Error en la lectura de la memoria FLASH.");
    #endif
    *write_address = 0x00000000;
    return false;
  }

  /* Si la ultima direccion es NULL, se inicia desde la posicion 0 de la memoria FLASH */
  if (*write_address == 0xFFFFFFFF) {
    *write_address = 0x00000000;
    #ifdef DEBUG_FLASH
    Serial.println("DEBUG (get_address): Iniciando escritura en la direccion 0.");
    #endif
  }

  /* Retornar si no ocurren errores */
  return true;
}

/************************************************************************************************************
 * @fn      start_flash
 * @brief   Inicia la memoria Flash
 * @param   NONE
 * @return  true sin error
 *          false error al iniciar FLASH
 */
bool start_flash(void)
{
  uint8_t iter_counter = 0;   /* Contador de iteraciones */

  flashTransport.begin();

  #ifdef DEBUG_FLASH_INFO
  if (flashTransport.supportQuadMode()) {
    Serial.println("DEBUG (start_flash): La memoria Flash soporta QSPI.");
  } else {
    Serial.println("DEBUG (start_flash): La memoria Flash no soporta QSPI.");
  }
  #endif

  /* Intentar iniciar FLASH QSPI */
  for ( iter_counter = 0; iter_counter <= MAX_ITERATIONS ; iter_counter ++) {
    /* Verificar si el FLASH QSPI ya inicio */
    if ( Flash_QSPI.begin() == true ) {
      #ifdef DEBUG_FLASH
      Serial.println("DEBUG (start_flash): Memoria flash iniciada correctamente.");
      #ifdef DEBUG_FLASH_INFO
      if (Flash_QSPI.isReady()) {
        Serial.println("La mem flash Está lista");
        Serial.print("ID JEDEC de la memoria flash: 0x");
        Serial.println(Flash_QSPI.getJEDECID(), HEX);  
      }
      #endif
      Serial.println("Memoria flash_QSPI inicializada correctamente");
      #endif
      return true;
    }

    #ifdef DEBUG_FLASH_INFO
    Serial.println("ERROR (start_flash): Error al inicializar la memoria Flash.");
    #endif
    delay(1000);

  }

  /* Si no se pudo iniciar en el numero maximo de intentos */
  #ifdef DEBUG_FLASH
  Serial.println("ERROR (start_flash): No se pudo iniciar la memoria Flash.");
  #endif
  return false;
}