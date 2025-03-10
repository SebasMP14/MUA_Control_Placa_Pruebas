/**
 * @file obc_comm.h
 * Lógica de comunicación con el OBC. Se aplica una FSM (Finite State Machine) de Moore
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024-2025> (github)
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
#define TRAMA_SIZE                  45
#define TRAMA_COMM                  6
#define MISSION_ID                  0x26
/*** Modos de operación ***/
#define ID_STANDBY                  0x00
#define ID_COUNT_MODE               0x01
#define ID_TRANSFER_MODE            0x02
#define ID_SENT_DATA                0x03
#define ID_TRANSFER_SYSINFO_MODE    0x09
#define ACK_MUA_TO_OBC              0x07


enum OperationMode {
    INICIO,
    STAND_BY,
    COUNT_MODE,
    TRANSFER_DATA_MODE,
    TRANSFER_INFO_MODE,
    FINISH,
    UNKNOWN_MODE
};
extern OperationMode currentMode;
extern bool setup_state;
extern RTC_SAMD51 rtc;
extern unsigned long timeOUT;
extern unsigned long timeOUT_invalid_frame;

extern uint8_t ack_MUA_to_OBC[TRAMA_COMM];
extern const uint8_t nack_MUA_to_OBC[TRAMA_COMM];
extern const uint8_t nack_IF_MUA_to_OBC[TRAMA_COMM];
extern const uint16_t crc_table[256];

void requestOperationMode(void);
void getTimestampFromGPS(void);
unsigned long getTime(void);
uint16_t crc_calculate(uint8_t *data);
void transferData(uint16_t Quantity);

#endif