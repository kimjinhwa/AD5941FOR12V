#include "batDeviceInterface.h"

BatDeviceInterface::BatDeviceInterface(){
        batVoltageAdcValue = 0.0;
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
        //adc_chars =(esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
        // ADC 설정 초기화 (예: ADC1, 12비트 해상도, 11dB 감쇠, 1100mV Vref)
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);


};
float BatDeviceInterface::readBatAdcValue(float filter)
{
  batVoltageAdcValue =0;
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  //if (batVoltageAdcValue == 0.0) // 처음 읽는 것이라면 
  {
    for (int i = 0; i < filter; i++)
    {
      batVoltageAdcValue +=  adc1_get_raw(ADC1_CHANNEL_0);
      //vTaskDelay(1);
    }
    batVoltageAdcValue = batVoltageAdcValue/filter ;
  }
  batVoltageAdcValue += VOLTAGE_OFFSET;
  uint32_t voltage = esp_adc_cal_raw_to_voltage((uint32_t)batVoltageAdcValue , &adc_chars);
  batVoltageAdcValue = voltage*((5.1+0.05567+1)/1);
  batVoltageAdcValue  /= 1000.0;
  if(batVoltageAdcValue < 1.3 ) batVoltageAdcValue = 0; 
// 이후, adc_chars를 esp_adc_cal_raw_to_voltage() 함수와 함께 사용
//   else  //이미 읽고 있다면.. 
//   {
//     uint16_t val =0;
//     for (int i = 0; i < filter / 2; i++)  // 처음 읽는 것의 반 만 읽자.
//     {
//       val = adc1_get_raw(ADC1_CHANNEL_0);
//       batVoltageAdcValue = (batVoltageAdcValue * (1.0 - 1.0 / filter) + val * 1.0 / filter);
//       vTaskDelay(1);
//     }
  //};
  //float retValue = 
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