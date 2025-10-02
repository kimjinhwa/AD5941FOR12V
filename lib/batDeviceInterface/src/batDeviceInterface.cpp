#include "batDeviceInterface.h"
#include "maingrobal.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_log.h>

BatDeviceInterface::BatDeviceInterface()
{
  batVoltageAdcValue = 0.0;
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
};

// ADC 보정 함수 - 실제 측정값 기반으로 보정
uint32_t BatDeviceInterface::adcCalibration(uint32_t adcVoltage)
{
  // 현재 측정값: 2.93V (실제) vs 2.283V (ESP32 ADC)
  // 보정 계수: 2.93 / 2.283 = 1.283
  
  // 더 정확한 보정을 위해 다항식 사용
  //float correctionFactor = 0;//0.011067608f;
  // if (adcVoltage < 1000) {
  //   // 저전압 구간: 선형 보정
  //   return (uint32_t)(adcVoltage * (1.283f+correctionFactor));
  // } else if (adcVoltage < 2000) {
  //   // 중전압 구간: 약간의 비선형 보정
  //   return (uint32_t)(adcVoltage * (1.285f+correctionFactor));
  // } else {
  //   // 고전압 구간: 더 큰 보정
  //   return (uint32_t)(adcVoltage * (1.290f+correctionFactor));
  // }
  return adcVoltage+30;
}

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
  uint32_t singleVoltage;
  //if (batVoltageAdcValue == 0.0) // 처음 읽는 것이라면 
  {
    for (int i = 0; i < filter; i++)
    {
      rValue = adc1_get_raw(ADC1_CHANNEL_0);
      singleVoltage = esp_adc_cal_raw_to_voltage((uint32_t)rValue , &adc_chars);
      // ADC 보정 함수 적용
      //singleVoltage = adcCalibration(singleVoltage);
      batVoltageAdcValue += singleVoltage;
      vTaskDelay(1);
    }
    // printf("rValue---->0 : %d\n",rValue);
    // printf("singleVoltage---->1 : %d\n",singleVoltage);
    batVoltageAdcValue = batVoltageAdcValue/filter ;
    //printf("batVoltageAdcValue---->2 : %f\n",batVoltageAdcValue);
  }
  batVoltageAdcValue  -= 20.0;  // 0.685V offset from SSR
  uint32_t voltage = batVoltageAdcValue;
  
  batVoltageAdcValue = voltage*5.0;  // OPAMP 배율
  batVoltageAdcValue += systemDefaultValue.voltageCompensation[_cellNumbver-1]/1000.0;
  //printf("batVoltageAdcValue---->3 : %f\n",batVoltageAdcValue );
  batVoltageAdcValue  /= 1000.0;  // mv -> v
  //printf("singleVoltage---->3 : %f\n",batVoltageAdcValue );
  //printf("singleVoltage---->4 : %f\n",batVoltageAdcValue );
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