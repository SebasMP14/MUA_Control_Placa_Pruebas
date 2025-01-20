#include "ads1260_driver.h"

ADS1260::ADS1260(SPIClass* spiInterface, uint8_t chip_select) : spi(spiInterface), chipSelectPin(chip_select) {
  pinMode(chip_select, OUTPUT);
  digitalWrite(chip_select, HIGH); // Deselecciona el ADC por defecto
}

void ADS1260::begin(void) {
  spi->begin();             // Inicializa la interfaz SPI
  // ADS1260::reset();
}

void ADS1260::setStartPin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); // Asegura que el pin START esté inicialmente apagado
}


//////////////////////////////////
// Use the NOP command to validate the CRC response byte and error detection without affecting normal operation
void ADS1260::noOperation(void) {
  sendCommand(ADS1260_NOP);
}

//  After reset, the conversion starts 512 / fCLK cycles later.
void ADS1260::reset(void) {
  sendCommand(ADS1260_RESET);
  delay(10); // Tiempo de reinicio recomendado en el datasheet
}

void ADS1260::start(void) {
  sendCommand(ADS1260_START);
}

void ADS1260::stop() {
  sendCommand(ADS1260_STOP);
}

uint32_t ADS1260::readData(uint8_t p_pin, uint8_t n_pin) {
  uint8_t inpmuxValue = (p_pin << 4) | n_pin;
  writeRegister(ADS1260_INPMUX, inpmuxValue); // Configuración de multiplexor
  delayMicroseconds(10);

  spi->beginTransaction( SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE1) );

  digitalWrite(chipSelectPin, LOW);
  uint8_t response1 = spi->transfer(ADS1260_RDATA);  // Selección de registro 
  uint8_t response2 = spi->transfer(ADS1260_DUMMY);
  uint8_t status = spi->transfer(ADS1260_DUMMY);
  uint8_t MSBdata = spi->transfer(ADS1260_DUMMY);
  uint8_t MIDdata = spi->transfer(ADS1260_DUMMY);
  uint8_t LSBdata = spi->transfer(ADS1260_DUMMY);
  digitalWrite(chipSelectPin, HIGH);

  spi->endTransaction();

  #ifdef DEBUG_ADS
  Serial.print("DEBUG (readData) -> response1: 0x");
  Serial.print(response1, HEX);
  Serial.print(", response2: 0x");
  Serial.println(response2, HEX);
  Serial.print("DEBUG (readData) -> Status 0x");
  Serial.println(status);
  #endif

  return (MSBdata << 16) | (MIDdata << 8) | LSBdata;
}

void ADS1260::sysOffsetCalibration(uint8_t shorted1, uint8_t shorted2) {
  // Configura las entradas para calibración
  uint8_t inpmuxValue = (shorted1 << 4) | shorted2;
  writeRegister(ADS1260_INPMUX, inpmuxValue);
  sendCommand(ADS1260_SYOCAL);
  delayMicroseconds(50); // Tiempo estimado para calibración
}

void ADS1260::gainCalibration(uint8_t vcc_pin, uint8_t vss_pin) {
  // Configura las entradas para calibración de ganancia
  uint8_t inpmuxValue = (vcc_pin << 4) | vss_pin;
  writeRegister(ADS1260_INPMUX, inpmuxValue);
  sendCommand(ADS1260_GANCAL);
  delay(500); // Tiempo estimado para calibración
}

void ADS1260::selfOffsetCalibration() {
  sendCommand(ADS1260_SFOCAL);
  delay(500); // Tiempo estimado para calibración
}

uint32_t ADS1260::readRegisterData(uint8_t reg) {
  return readRegister(reg);
}

void ADS1260::writeRegisterData(uint8_t reg, uint8_t data) { // 
  writeRegister(reg, data);
  ///// Agregar comprobación
}

void ADS1260::registerLock() { // Page 50
  sendCommand(ADS1260_LOCK);
}

void ADS1260::registerUnlock() { // Page 50
  sendCommand(ADS1260_UNLOCK);
}

////////////////////////
void ADS1260::sendCommand(uint8_t command) {
  spi->beginTransaction( SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE1) );

  digitalWrite(chipSelectPin, LOW);
  uint8_t response1 = spi->transfer(command);
  uint8_t response2 = spi->transfer(ADS1260_DUMMY);
  digitalWrite(chipSelectPin, HIGH);

  spi->endTransaction();

  #ifdef DEBUG_ADS
  Serial.print("DEBUG (sendCommand) -> response1: 0x");
  Serial.print(response1, HEX);
  Serial.print(", response2: 0x");
  Serial.println(response2, HEX);
  #endif
}

uint8_t ADS1260::readRegister(uint8_t reg) { // Page 48
  spi->beginTransaction( SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE1) );

  digitalWrite(chipSelectPin, LOW);
  uint8_t response1 = spi->transfer(ADS1260_RREG | reg);  // Envia comando de lectura y se recibe 0xFF
  uint8_t response2 = spi->transfer(ADS1260_DUMMY);       // Ciclo adicional según el datasheet y se recibe Echo Byte 1
  uint8_t value = spi->transfer(ADS1260_DUMMY); // Se envía el ultimo byte 0x00 y se recibe Registerdata
  delay(1);
  digitalWrite(chipSelectPin, HIGH);

  spi->endTransaction();
  
  #ifdef DEBUG_ADS
  Serial.print("DEBUG (readRegister) -> response1: 0x");
  Serial.print(response1, HEX);
  Serial.print(", response2: 0x");
  Serial.println(response2, HEX);
  #endif

  return value;
}

void ADS1260::writeRegister(uint8_t reg, uint8_t value) { // Page 49
  spi->beginTransaction( SPISettings(SPI_CLK_SPEED, MSBFIRST, SPI_MODE1) );

  digitalWrite(chipSelectPin, LOW);
  uint8_t response1 = spi->transfer(ADS1260_WREG | reg);  // Selección de registro 
  uint8_t response2 = spi->transfer(value);               
  digitalWrite(chipSelectPin, HIGH);

  spi->endTransaction();

  #ifdef DEBUG_ADS
  Serial.print("DEBUG (writeRegister) -> response1: 0x");
  Serial.print(response1, HEX);
  Serial.print(", response2: 0x");
  Serial.println(response2, HEX);
  #endif
}