#ifndef MCP4561_DRIVER_H
#define MCP4561_DRIVER_H

// #define DEBUG_MCP

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MCP_ADDRESS 0b00101110 // 0x2E:startBit, A6, ..., A0, 

extern const float V_to_umbral;

bool writeMCP0(uint8_t valor);
bool writeMCP1(uint8_t valor);
uint16_t readMCP0(void);
uint16_t readMCP1(void);

uint8_t VMCP_to_DEC(float voltage);

#endif

// https://github.com/SteveQuinn1/MCP4561_DIGI_POT/blob/master/examples/MCP4561_GenericPotCommands/MCP4561_GenericPotCommands.ino