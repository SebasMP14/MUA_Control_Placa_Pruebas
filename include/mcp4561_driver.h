#ifndef MCP4561_DRIVER_H
#define MCP4561_DRIVER_H

// #define DEBUG_MCP

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MCP_ADDRESS 0b00101110 // 0x2E:startBit, A6, ..., A0, 

bool writeMCP0(uint8_t valor);
bool writeMCP1(uint8_t valor);
uint16_t readMCP0(void);
uint16_t readMCP1(void);

#endif

// https://github.com/SteveQuinn1/MCP4561_DIGI_POT/blob/master/examples/MCP4561_GenericPotCommands/MCP4561_GenericPotCommands.ino