#ifndef CALCULOS_H
#define CALCULOS_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>

#include "max1932_driver.h"

// extern Adafruit_ADS1115 ads;

#define DEBUG_CALCULOS
#define PI 3.1415926535897932384626433832795

extern float Fc;                    // Frecuencia de corte en Hz (ajustable)
extern float Fs;                    // Frecuencia de muestreo en Hz (ajustable)
extern float a1, a2, b0, b1, b2;    // Coeficientes del filtro Butterworth

extern float x[3];                         // Últimos 3 valores de entrada (corriente)
extern float y[3];                         // Últimos 3 valores de salida (corriente filtrada)

void init_butterworth(void);
float* apply_butterworth(float *input, uint8_t Elementos);
// float apply_butterworth(float new_input);
float obtain_Vbd(float *inverseCurrent_I, float *inverseVoltage, uint8_t Elementos);
float Vbd_teorical(float Temperature);


#endif