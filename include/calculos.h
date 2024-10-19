#ifndef CALCULOS_H
#define CALCULOS_H

#include <math.h>

#define DEBUG_CALCULOS
#define PI 3.1415926535897932384626433832795

extern float Fc;                    // Frecuencia de corte en Hz (ajustable)
extern float Fs;                    // Frecuencia de muestreo en Hz (ajustable)
extern float a1, a2, b0, b1, b2;    // Coeficientes del filtro Butterworth

float x[3];                         // Últimos 3 valores de entrada (corriente)
float y[3];                         // Últimos 3 valores de salida (corriente filtrada)

void ini_butterworth(void);
float apply_butterworth(float new_input);

#endif