#ifndef _BAT_DEVICEINTERFACE_H
#define _BAT_DEVICEINTERFACE_H

#include "Arduino.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <stdlib.h>
#define VOLTAGE_OFFSET 2.0  //저항의 오차값 정도라고 보다 
class BatDeviceInterface
{
    public:
    BatDeviceInterface();
    float readBatAdcValue(uint16_t cellNumbver, float filter);
    float getBatVoltage(float batVoltageAdcValue);
    float batVoltageAdcValue ;
    private:
        float readBatAdcValue(float filter);
        esp_adc_cal_characteristics_t adc_chars;
        uint16_t _cellNumbver;
};

#endif