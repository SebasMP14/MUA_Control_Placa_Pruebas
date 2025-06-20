#include "mcp4561_driver.h"

const float V_to_umbral = 0.7;

bool writeMCP0(uint8_t valor) {
  // uint8_t DH = valor/256;
  uint8_t VW0 = 0b00000000 ;//|| DH;
  uint8_t D2 = valor;
  #ifdef DEBUG_MCP
  Serial.print("DEBUG (escribirPot) -> VW0: "); Serial.println(VW0, BIN);
  Serial.print("DEBUG (escribirPot) -> D2: "); Serial.println(D2, HEX);
  #endif

  Wire.beginTransmission(MCP_ADDRESS); //Dirección del MCP4561
  
  Wire.write(VW0);
  Wire.write(D2);

  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    #ifdef DEBUG_MCP
    Serial.print("ERROR (writeMCP) -> Error en la transmisión I2C - Wiper0: ");
    Serial.println(status);
    #endif
    return false;  // Detener si hay error en la transmisión
  }

  // if ( valor == readMCP0() )  return true;  // Configuración y verificación exitosa
  return true;
}
bool writeMCP1(uint8_t valor) {
  uint8_t VW1 = 0b00000001 ;
  uint8_t D2 = valor;
  #ifdef DEBUG_MCP
  Serial.print("DEBUG (escribirPot) -> VW1: "); Serial.println(VW1, BIN);
  Serial.print("DEBUG (escribirPot) -> D2: "); Serial.println(D2, HEX);
  #endif

  Wire.beginTransmission(MCP_ADDRESS); //Dirección del MCP4561
  
  Wire.write(VW1);
  Wire.write(D2);

  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    #ifdef DEBUG_MCP
    Serial.print("ERROR (writeMCP) -> Error en la transmisión I2C - Wiper1: ");
    Serial.println(status);
    #endif
    return false;  // Detener si hay error en la transmisión
  }

  return true;  // Configuración exitosa
}

uint16_t readMCP0(void) {
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(0b00001100);  // Dirección del registro Wiper 0
  Wire.endTransmission(false);

  Wire.requestFrom(MCP_ADDRESS, 2); // Solicita 2 bytes
  delay(1);
  if (Wire.available() < 2) {
    return 0xFFFF;
  }

  uint8_t HSB = Wire.read();
  uint8_t LSB = Wire.read();
  return (uint16_t)( HSB << 8 | LSB ) & 0x01FF;
}

uint16_t readMCP1(void) {
  Wire.beginTransmission(MCP_ADDRESS);
  Wire.write(0b00001101);  // Dirección del registro Wiper 1
  Wire.endTransmission(false);

  Wire.requestFrom(MCP_ADDRESS, 2); // Solicita 2 bytes
  delay(1);
  if (Wire.available() < 2) {
    return 0xFFFF;
  }

  uint8_t HSB = Wire.read();
  uint8_t LSB = Wire.read();
  return (uint16_t)( HSB << 8 | LSB ) & 0x01FF;
}

uint8_t VMCP_to_DEC(float voltage) {
  return (uint8_t)((voltage - V_to_umbral) * 0xFF / 4.096);
}