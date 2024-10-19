/**
 * tmp100_driver.cpp
 * Funciones de configuración y lectura del sensor de temperatura TMP100 para la misión M.U.A. de FIUNA 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - 
 */
#include "tmp100_driver.h"

/************************************************************************************************************
 * @fn      start_tmp100
 * @brief   Establece configuración para las lecturas de temperatura
 * @param   NONE
 * @return  true exitoso - false fallido
 * configRegister:  0b  OS/ALERT: One-shot measurement - 1
 * R1  R0: 0 0 - 9 bits(0.5ºC, 40 ms) | 0 1 - 10 bits(0.25ºC, 80 ms) | 1 0 - 11 bits(0.125ºC, 160 ms) | 1 1 - 12 bits(0.0625ºC, 320 ms)
 * F1  F0: 0 0 - 1 fallo | 0 1 - 2 fallos
 * POL: Polarity for TMP101 - 0
 * TM: Thermostate mode - 0 Comparator mode - 1 interrupt mode
 * SD: Shut-down mode 1
 */
bool start_tmp100(void) {
	uint8_t configRegister = 0b01100000;  // Default configuration 12 bits

	Wire.begin();
	// Wire.setClock(400000); // Velocidad 400 KHz 
	Wire.setClock(2000000); 	// Velocidad de 2 MHz

  /* Activación de HIGH SPEED MODE: status = 3 esperado*/
	Wire.beginTransmission(TMP100_ADDRESS);
	Wire.write(0b00001000);
	uint8_t status = Wire.endTransmission();
	if (status != 3) { // 3 para HIGH SPEED
		#ifdef DEBUG_TMP
		Serial.print("ERROR (start_tmp100) HSM: Error en la transmisión I2C: ");
		Serial.println(status);
		return false;  // Detener si hay error en la transmisión
		#endif
	}

	Wire.beginTransmission(TMP100_ADDRESS);
	Wire.write(0x01);  // Puntero al registro de configuración

	// Establecer resolución basada en el valor de 9 a 12 bits
	switch (resolution) {
		case 9:
			configRegister = 0b00000000;  // 9 bits
			break;
		case 10:
			configRegister = 0b00100000;  // 10 bits
			break;
		case 11:
			configRegister = 0b01000000;  // 11 bits
			break;
		case 12:
			configRegister = 0b01100000;  // 12 bits
			break;
		default:
			configRegister = configRegister;
			#ifdef DEBUG_TMP
			Serial.println("ERROR (start_tmp100): Resolución no válida, debe estar entre 9 y 12.");
			#endif
	}

	Wire.write(configRegister);

	status = Wire.endTransmission();
	if (status != 0) {
		#ifdef DEBUG_TMP
		Serial.print("ERROR (start_tmp100): Error en la transmisión I2C: ");
		Serial.println(status);
		return false;  // Detener si hay error en la transmisión
		#endif
	}

	return true;  // Configuración exitosa
}

/************************************************************************************************************
 * @fn      read_tmp100
 * @brief   Obtiene la lectura del sensor de temperatura TMP100
 * @param   NONE
 * @return  float exitoso - NAN fallido
 */
float read_tmp100(void) {
  Wire.beginTransmission(TMP100_ADDRESS);
  Wire.write(0x00);  // Puntero al registro de temperatura
  uint8_t status = Wire.endTransmission();
	if (status != 0) {
		#ifdef DEBUG_TMP
		Serial.print("ERROR (start_tmp100): Error en la transmisión I2C: ");
		Serial.println(status);
		return false;  // Detener si hay error en la transmisión
		#endif
	}

  Wire.requestFrom(TMP100_ADDRESS, 2);  // Solicita 2 bytes (MSB y LSB)
  if ( Wire.available() == 2 ) {
		int msb = Wire.read();  // Primer byte (MSB)
		int lsb = Wire.read();  // Segundo byte (LSB)
		
		// Combinar MSB y LSB en base a la resolución
		int16_t rawTemp = 0;
		switch (resolution) {
			case 9:
				rawTemp = (msb << 1) | (lsb >> 7);  // Solo se usa 1 bit del LSB
				break;
			case 10:
				rawTemp = (msb << 2) | (lsb >> 6);  // Se usan 2 bits del LSB
				break;
			case 11:
				rawTemp = (msb << 3) | (lsb >> 5);  // Se usan 3 bits del LSB
				break;
			case 12:
				rawTemp = (msb << 4) | (lsb >> 4);  // Se usan 4 bits del LSB (12 bits)
				break;
			default:
				rawTemp = (msb << 4) | (lsb >> 4);  // Se usan 4 bits del LSB (12 bits)
				break;
		}

		// Ajustar el valor para negativos (sign extension)
		if (rawTemp > (1 << (resolution - 1)) - 1) {
				rawTemp |= ~((1 << resolution) - 1);  // Para valores negativos
		}
		/* 	El TMP10X tiene un registro de 16 bits, de los cuáles los 12 bits MSB son relevantes para la mayor
		resolución. Estos 12 bits (con signo) dan un valor entero, este se multiplica por el factor correspondiente
		a cada resolución mostrado abajo para tener el valor en grados celcius.	*/
		switch (resolution) {
			case 9:
				return rawTemp * 0.5;  // Conversión a Celsius para 9 bits
			case 10:
				return rawTemp * 0.25;  // Conversión a Celsius para 10 bits
			case 11:
				return rawTemp * 0.125;  // Conversión a Celsius para 11 bits
			case 12:
			default:
				return rawTemp * 0.0625;  // Conversión a Celsius para 12 bits
		}
	}

  return NAN;  // Si no se recibe la respuesta
}