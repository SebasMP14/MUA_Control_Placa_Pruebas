#include "calculos.h"

// Adafruit_ADS1115 ads;

float Fc = 5.0;                 // Frecuencia de corte en Hz (ajustable)
float Fs = 100.0;               // Frecuencia de muestreo en Hz (ajustable)
float a1, a2, b0, b1, b2;       // Coeficientes del filtro Butterworth
float x[3] = {0.0, 0.0, 0.0};     // Últimos valores de entrada
float y[3] = {0.0, 0.0, 0.0};     // Últimos valores de salida

/************************************************************************************************************
 * @fn      init_butterworth
 * @brief   
 * @param   
 * @return  
 */
void init_butterworth(void) {
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

  float norm = 1 + a1 + a2;     // Normalización de coeficientes

  b0 /= norm;
  b1 /= norm;
  b2 /= norm;
  a1 /= norm;
  a2 /= norm;
}

/************************************************************************************************************
 * @fn      
 * @brief   
 * @param   
 * @return  
 */
float* apply_butterworth(float *input, uint8_t Elementos) {
  // Crear un nuevo arreglo para almacenar la salida filtrada
  float *filtered_output = (float*)malloc(Elementos * sizeof(float));
  if (filtered_output == NULL) {
    Serial.println("Error: No se pudo asignar memoria para la salida del filtro.");
    return NULL;
  }

  // Reiniciar las variables del filtro
  for (int i = 0; i < 3; i++) {
    x[i] = 0.0;
    y[i] = 0.0;
  }

  // Aplicar el filtro a cada elemento
  for (uint8_t i = 0; i < Elementos; i++) {
    x[2] = x[1];
    x[1] = x[0];
    x[0] = input[i];

    y[2] = y[1];
    y[1] = y[0];
    y[0] = b0 * x[0] + b1 * x[1] + b2 * x[2] - a1 * y[1] - a2 * y[2];

    filtered_output[i] = y[0];
  }

  return filtered_output;
}

/************************************************************************************************************
 * @fn      obtain_Vbias
 * @brief   Se obtiene la curva I-V inversa del SiPM, se aplica la inversa de la derivada del logaritmo neperiano de la curva. El pico de
 *          esta es el Voltaje de ruptura, sumandole el over voltage tenemos Vbias.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @return  ---todo
 */
float obtain_Vbd(float *inverseCurrent_I, float *inverseVoltage, uint8_t Elementos) {
  float logarithmicCurrent[Elementos];
  float derivative[Elementos];
  float inverseDerivative[Elementos];
  float BreakdownVoltage = inverseVoltage[0];
  int indexPeak = 0;

  for ( uint8_t i = 0; i < Elementos; i++ ) {            // Logaritmo natural de 
    logarithmicCurrent[i] = logf(inverseCurrent_I[i]);
  }

  for ( uint8_t i = 1; i < Elementos - 1; i++ ) {
    derivative[i] = (logarithmicCurrent[i+1] - logarithmicCurrent[i-1]) / 
                    (inverseVoltage[i+1] - inverseVoltage[i-1]);
    inverseDerivative[i] = 1.0 / derivative[i];

    if ( inverseDerivative[i] > BreakdownVoltage ) {
      BreakdownVoltage = inverseDerivative[i];
      indexPeak = i;
    }
  }

  return inverseVoltage[indexPeak];
}

/************************************************************************************************************
 * @fn      Vbd_teorical
 * @brief   Voltaje de ruptura teorica de acuerdo a la temperatura
 * @param   Temperature
 * @return  valor del voltaje estimado
 */
float Vbd_teorical(float Temperature) {   // Celcius
  return 23.9985 + 0.0215 * Temperature;  // Curva lineal
}

/************************************************************************************************************
 * @fn      V_to_command
 * @brief   Conversión de voltaje a commando bin para el MAX
 * @param   Voltaje: Es el voltaje objetivo en la salida del MAX
 * @return  comando en binario
 */
