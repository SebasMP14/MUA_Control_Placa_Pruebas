/**
 * calculos.h
 *  
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME → Muon_Boys
 * 
 * Made by:
 * - Est. Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * - 
 */
#ifndef CALCULOS_H
#define CALCULOS_H

#include <Arduino.h>
#include <math.h>

#define DEBUG_CALCULOS
#define PI 3.1415926535897932384626433832795

extern float Fc;                    // Frecuencia de corte en Hz (ajustable)
extern float Fs;                    // Frecuencia de muestreo en Hz (ajustable)
extern float a1, a2, b0, b1, b2;    // Coeficientes del filtro Butterworth

extern float x[3];                         // Últimos 3 valores de entrada (corriente)
extern float y[3];                         // Últimos 3 valores de salida (corriente filtrada)

extern const float Voffset;
extern float firstCurrent1;
extern float firstCurrent2;
extern const float ResisB; 

float* moving_average(float *input, uint16_t Elementos, uint8_t window_size);
void sliding_moving_average(float *input, uint16_t N, uint8_t M, float* output); // se usa
void init_butterworth(void);
float* apply_butterworth(float *input, uint16_t Elementos);
// float apply_butterworth(float new_input);
float obtain_Vbd(float *inverseCurrent_I, float *inverseVoltage, uint16_t Elemento, float* Vcurr, uint16_t* indexPeak); // se usa
float Vbd_teorical(float Temperature); // se usa

float SiPMCurrent(float VCurrent, float firstCurrent);
float VMAX_OUT(uint8_t CMD_MAX, uint16_t CMD_DAC);
float Vtia(float VCurrent, float firstCurrent);

#endif