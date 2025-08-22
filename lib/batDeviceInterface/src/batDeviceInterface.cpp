#include "batDeviceInterface.h"
#include "maingrobal.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

BatDeviceInterface::BatDeviceInterface(){
        batVoltageAdcValue = 0.0;
        adc1_config_width(ADC_WIDTH_BIT_12);
        //esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);


};

float BatDeviceInterface::readBatAdcValue(uint16_t cellNumbver, float filter)
{
  this->_cellNumbver = cellNumbver;
  if(this->_cellNumbver<1 || this->_cellNumbver > systemDefaultValue.installed_cells )
  {
    ESP_LOGE("Error", "Bat number is : %d ", this->_cellNumbver);
    return 0.0f;
  }
  return readBatAdcValue(filter);
}

float BatDeviceInterface::readBatAdcValueExt(float filter)
{
  uint32_t rValue = 0;
  batVoltageAdcValue =0;

  // esp_adc_cal_characteristics_t adc_chars;
  // esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    for (int i = 0; i < filter; i++)
    {
      //batVoltageAdcValue +=  adc1_get_raw(ADC1_CHANNEL_0);
      rValue  = analogRead(ADC1_CHANNEL_0);
      batVoltageAdcValue  += esp_adc_cal_raw_to_voltage((uint32_t)batVoltageAdcValue , &adc_chars);;
      vTaskDelay(1);
    }
    batVoltageAdcValue = batVoltageAdcValue/filter ;
    batVoltageAdcValue += systemDefaultValue.voltageCompensation[_cellNumbver-1];
  uint32_t voltage = batVoltageAdcValue; 
  
  batVoltageAdcValue = voltage*100.0/33.0*2.0  ;
  batVoltageAdcValue  /= 1000.0;
  if(batVoltageAdcValue < 1.3 ) batVoltageAdcValue = 0; 
  return batVoltageAdcValue ;
}
float BatDeviceInterface::readBatAdcValue(float filter)
{
  uint32_t rValue = 0;
  batVoltageAdcValue =0;

  //if (batVoltageAdcValue == 0.0) // 처음 읽는 것이라면 
  {
    for (int i = 0; i < filter; i++)
    {
      rValue = analogRead(ADC1_CHANNEL_0);
      batVoltageAdcValue +=  esp_adc_cal_raw_to_voltage((uint32_t)rValue , &adc_chars);
      vTaskDelay(1);
    }
    batVoltageAdcValue = batVoltageAdcValue/filter ;
  }
  batVoltageAdcValue += systemDefaultValue.voltageCompensation[_cellNumbver-1];
  uint32_t voltage = batVoltageAdcValue;
  
  batVoltageAdcValue = voltage*100.0/33.0*2.0  ;
  batVoltageAdcValue  /= 1000.0;
  if(batVoltageAdcValue < 1.3 ) batVoltageAdcValue = 0; // 1 offset = 0.00151V
  return batVoltageAdcValue ;
}
float BatDeviceInterface::getBatVoltage(float batVoltageAdcValue)
{
  //*bat_result = gVolts*2; //리튬전지까지 사용하기 위하여 저항을 3:1변경하여 2배수를 해준다
  float volts;
  if (batVoltageAdcValue > 3000)
  {
    volts = 0.0005 * batVoltageAdcValue + 1.0874;
  }
  else if (batVoltageAdcValue <= 3000 && batVoltageAdcValue > 2500)
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372 + (batVoltageAdcValue / 51.0 - 10.0) * 0.00065;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  else if (batVoltageAdcValue <= 2500 && batVoltageAdcValue > 2000)
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372 + (batVoltageAdcValue / 51.0 - 10.0) * 0.00065;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  else if (batVoltageAdcValue <= 2000 && batVoltageAdcValue > 1500)
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372 + (batVoltageAdcValue / 51.0 - 10.0) * 0.0005;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  else if (batVoltageAdcValue <= 1500 && batVoltageAdcValue > 1000)
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372 + (batVoltageAdcValue / 51.0 - 10.0) * 0.0005;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  else if (batVoltageAdcValue <= 1000 && batVoltageAdcValue > 550)
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372 + (batVoltageAdcValue / 51.0 - 10.0) * 0.001;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  else
  {
    volts = 0.0008 * batVoltageAdcValue + 0.1372;
    volts += 0.001199999 * 100 * (batVoltageAdcValue / 4095.0); // 보정값
  }
  //*batVal = 2 * volts;
  return volts + VOLTAGE_OFFSET/1000.0 ;
}