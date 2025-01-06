/**
 * obc_comm.h
 * Lógica de comunicación con el OBC. Se aplica una FSM (Finite State Machine) de Moore
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

#ifndef OBC_COMM_H
#define OBC_COMM_H

#include "Arduino.h"
#include "RTC_SAMD51.h"
#include "flash_driver.h"

#define DEBUG_OBC
#define TRAMA_SIZE          44
/*** Modos de operación ***/
#define ID_STANDBY          0x00
#define ID_COUNT_MODE       0x01
#define ID_TRANSFER_MODE    0x02
#define ACK_MUA_TO_OBC      0x07


enum OperationMode {
    INICIO,
    STAND_BY,
    COUNT_MODE,
    TRANSFER_DATA_MODE,
    FINISH,
    TRANSFER_INFO_MODE,
    UNKNOWN_MODE
};
extern OperationMode currentMode;
extern bool setup_state;
extern RTC_SAMD51 rtc;
 //  = {0x07, 0x01, 0x00};
extern unsigned long timeOUT;

void requestOperationMode(void);
void getTimestampFromGPS(void);
unsigned long getTime(void);
uint16_t calcularCRC(const uint8_t *data, size_t length);
void transferData(uint16_t Quantity);

#endif