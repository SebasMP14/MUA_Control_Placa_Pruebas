/**
 * calculos.cpp
 *  
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME → Muon_Boys
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - 
 */
#include "calculos.h"

float Fc = 5.0;                 // Frecuencia de corte en Hz (ajustable)
float Fs = 100.0;               // Frecuencia de muestreo en Hz (ajustable)
float a1, a2, b0, b1, b2;       // Coeficientes del filtro Butterworth
float x[3] = {0.0, 0.0, 0.0};     // Últimos valores de entrada
float y[3] = {0.0, 0.0, 0.0};     // Últimos valores de salida

/************************************************************************************************************
 * @fn      sliding_moving_average
 * @brief   
 * @param   
 * @return  
 * TODO: - 
 */
void sliding_moving_average(float* input, uint16_t N, uint8_t M, float* output) {
  float accumulator = 0.0;
  uint8_t aux = (M - 1) / 2;

  // Para las primeras aux+1 muestras
  for ( uint8_t i = 0; i < M; i++ ) {
    accumulator += input[i];
    if ( i >= aux ) {
      output[i - aux] = accumulator / (i + 1);
    }
  }

  // Para el resto de las muestras (ventanas deslizantes completas)
  for ( uint16_t i = M - aux; i < N - aux; i++ ) {
    accumulator += input[i + aux] - input[i - aux - 1];  // Actualiza el acumulador
    output[i] = accumulator / M;  // Calcula el promedio
  }

  // Para los últimos valores
  for ( uint16_t i = N - aux; i < N; i++ ) {
    accumulator -= input[i - aux - 1];  // restar el valor sobrante
    output[i] = accumulator / (N - i + aux);  // Promedio con los valores restantes
  }
}


/************************************************************************************************************
 * @fn      
 * @brief   
 * @param   
 * @return  
 * TODO: - 
 */
float* moving_average(float *input, uint16_t Elementos, uint8_t window_size) {
  static float output[256];
  float sum = 0;

  if ( window_size > Elementos ) {
    window_size = Elementos; // Prevenir ventanas más grandes que la entrada
  }

  // Inicializar la ventana
  for ( uint8_t i = 0; i < window_size; i++ ) {
    sum += input[i];
    output[i] = sum / (i + 1);  // Promedio para las primeras muestras
  }

  // Calcular el promedio móvil
  for ( uint16_t i = window_size; i < Elementos; i++ ) {
    sum += input[i] - input[i - window_size];
    output[i] = sum / window_size;
  }

  return output;
}


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
 * TODO: - No usar malloc
 */
float* apply_butterworth(float *input, uint16_t Elementos) {
  // Crear un nuevo arreglo para almacenar la salida filtrada
  float *filtered_output = (float*)malloc(Elementos * sizeof(float));
  if (filtered_output == NULL) {
    #ifdef DEBUG_CALCULOS
    Serial.println("Error (apply_butterworth): No se pudo asignar memoria para la salida del filtro.");
    #endif
    return NULL;
  }

  // Reiniciar las variables del filtro
  for (int i = 0; i < 3; i++) {
    x[i] = 0.0;
    y[i] = 0.0;
  }

  // Aplicar el filtro a cada elemento
  for (uint16_t i = 0; i < Elementos; i++) {
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
 * @param   
 * @return  ---todo
 */
float obtain_Vbd(float *inverseCurrent, float *inverseVoltage, uint16_t Elementos, float *Vcurr, uint16_t *indexPeak) {
  #ifdef DEBUG_CALCULOS
  Serial.println("DEBUG (obtain_Vbd) -> Ejecutandose");
  #endif
  *indexPeak = 0;
  float minInverseDerivative = __FLT_MAX__; 
  float logarithmicCurrent[Elementos];
  float inverseDerivative[Elementos];

  // Logaritmo natural de la corriente
  for (uint16_t i = 0; i < Elementos; i++) {
    if (inverseCurrent[i] > 0.0f) { 
      logarithmicCurrent[i] = logf(inverseCurrent[i]);            // Calculo del log de la corriente
    } else {
      logarithmicCurrent[i] = -__FLT_MAX__; // Valor mínimo para descartar
    }
  }
  #ifdef DEBUG_CALCULOS
  Serial.println("DEBUG (obtain_Vbd) -> logaritmo calculado");
  #endif

  // Calculo de la derivada y su inversa
  for (uint16_t i = 1; i < Elementos - 1; i++) {
    float dV = inverseVoltage[i+1] - inverseVoltage[i-1];
    if (fabsf(dV) < 1e-6f) continue; // Evitar divisiones por valores muy pequeños

    float dLogI = logarithmicCurrent[i+1] - logarithmicCurrent[i-1];
    float derivative = dLogI / dV;

    if (derivative > 0.0f) { // Filtrar derivadas negativas
      inverseDerivative[i] = 1.0f / derivative;
      if (inverseDerivative[i] < minInverseDerivative) {
        minInverseDerivative = inverseDerivative[i];
        *indexPeak = i;
      }
    }
  }
  #ifdef DEBUG_CALCULOS
  Serial.print("DEBUG (obtain_Vbd) -> Vbd obtenido: ");
  Serial.println(inverseVoltage[*indexPeak], 6);
  #endif

  *Vcurr = inverseCurrent[*indexPeak];
  return inverseVoltage[*indexPeak];
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


/* anteriormente en obtain_Vbd
float logarithmicCurrent[Elementos];
  float derivative[Elementos];
  float inverseDerivative[Elementos];
  float BreakdownVoltage = inverseVoltage[0];
  uint16_t indexPeak = 0;

  for ( uint16_t i = 0; i < Elementos; i++ ) {            // Logaritmo natural de 
    logarithmicCurrent[i] = logf(inverseCurrent[i]);
  }

  for ( uint16_t i = 1; i < Elementos - 1; i++ ) {
    derivative[i] = (logarithmicCurrent[i+1] - logarithmicCurrent[i-1]) / 
                    (inverseVoltage[i+1] - inverseVoltage[i-1]);
    inverseDerivative[i] = 1.0 / derivative[i];

    if ( inverseDerivative[i] > BreakdownVoltage ) {
      BreakdownVoltage = inverseDerivative[i];
      indexPeak = i;
    }
  }
*/
