
#ifndef _MODBUSLCDMODULE_H
#define _MODBUSLCDMODULE_H
#include <Arduino.h> 
#include "mainGrobal.h"
#include <Stream.h>  
#include <queue>
#include <mutex>
#include <vector>

using std::queue;


void setupModbusAgentForLcd();
#endif