#ifndef OBC_COMM_H
#define OBC_COMM_H

#include "Arduino.h"
#include "RTC_SAMD51.h"

#define DEBUG_OBC

enum OperationMode {
    COUNT_MODE,
    TRANSFER_MODE,
    UNKNOWN_MODE
};
extern OperationMode currentMode;
extern bool setup_state;
extern RTC_SAMD51 rtc;

void requestOperationMode(void);
void getTimestampFromGPS(void);
unsigned long getTime(void);
void transferData(uint16_t Quantity);

#endif