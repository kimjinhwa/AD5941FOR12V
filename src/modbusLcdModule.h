
#ifndef _MODBUSLCDMODULE_H
#define _MODBUSLCDMODULE_H
#include <Arduino.h> 
#include "mainGrobal.h"
#include <Stream.h>  
#include <queue>
#include <mutex>
#include <vector>
#include "modbusRtu.h"

//using std::queue;
void modbusLcdSetup();
Error setDataToLcd(uint16_t  startAddress, uint8_t sendLength);
#endif