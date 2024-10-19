#include "calculos.h"

float Fc = 5.0;                 // Frecuencia de corte en Hz (ajustable)
float Fs = 100.0;               // Frecuencia de muestreo en Hz (ajustable)
float a1, a2, b0, b1, b2;       // Coeficientes del filtro Butterworth

void ini_butterworth(void) {
  float omega = 2 * PI * Fc / Fs;
  float sn = sin(omega);
  float cs = cos(omega);
  float alpha = sn / sqrt(2);   // Q factor para Butterworth de 2do orden
  
  // Coeficientes de Butterworth
  b0 = (1 - cs) / 2;
  b1 = 1 - cs;
  b2 = (1 - cs) / 2;
  a1 = -2 * cs;
  a2 = 1 - alpha;
}

float apply_butterworth(float new_input) {
  // Desplazar las muestras anteriores
  x[2] = x[1];
  x[1] = x[0];
  x[0] = new_input;

  y[2] = y[1];
  y[1] = y[0];

  // Filtro de segundo orden
  return b0 * x[0] + b1 * x[1] + b2 * x[2] - a1 * y[1] - a2 * y[2];
}