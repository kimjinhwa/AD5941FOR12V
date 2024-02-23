#include <Arduino.h>
#include <EEPROM.h>
#include "modbusRtu.h"
#include "mainGrobal.h"
#include <RtcDS1302.h>
void setRtcNewTime(RtcDateTime rtc);

ModbusMessage FC06(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];
  memset(sendValue, 0x00, 256);
  // get request values
  request.get(2, address);
  request.get(4, value);
  uint16_t writeAddress = (0x00FF & address );

  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  RtcDateTime now;
  now = RtcDateTime(tmv.tv_sec);

  if (writeAddress  > 255)
  {
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }
  response.add(request.getServerID(), request.getFunctionCode(), 2);
  response.add(value);

  if( writeAddress  <40){
    cellvalue[writeAddress  ].readTime = tmv.tv_sec;
    cellvalue[writeAddress  ].compensation = value;
  }
  else if (writeAddress  >= 40 && writeAddress  < 47)
  { // 시간을 설정한다.
    //Serial.printf("\nFunction code 06 %d[%d] %d ",address,writeAddress,value);
    switch (writeAddress  )
    {
    case 40:
      now = RtcDateTime(value, now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second());
      break;
    case 41:
      now = RtcDateTime(now.Year(), value, now.Day(), now.Hour(), now.Minute(), now.Second());
      break;
    case 42:
      now = RtcDateTime(now.Year(), now.Month(), value, now.Hour(), now.Minute(), now.Second());
      break;
    case 43:
      now = RtcDateTime(now.Year(), now.Month(), now.Day(), value, now.Minute(), now.Second());
      break;
    case 44:
      now = RtcDateTime(now.Year(), now.Month(), now.Day(), now.Hour(), value, now.Second());
      break;
    case 45:
      now = RtcDateTime(now.Year(), now.Month(), now.Day(), now.Hour(), now.Minute(), value);
      break;

    default:
      break;
    }
    tmv.tv_sec = now.TotalSeconds();
    tmv.tv_usec = 0;
    settimeofday(&tmv, NULL);
    setRtcNewTime(now);
  }
  return response;
};
ModbusMessage FC03(ModbusMessage request) {
  uint16_t address;           // requested register address
  uint16_t writeAddress;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];

  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  RtcDateTime now;
  now = RtcDateTime(tmv.tv_sec);
  memset(sendValue,0x00,256);
  // get request values
  request.get(2, address);
  request.get(4, words);

  if(  words ==0  ||  ((address & 0x00FF) + words) > 255){
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  } 
  
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  if(address < (0x00 + MAX_INSTALLED_CELLS)){  // 0~39번지의 요구이면 
    for (int i = address; i< words+address; i++)
    {
      value = (uint16_t)cellvalue[i].voltage*100;
      response.add(value);
    }
  }
  else if((address >= 0x100) && address+words < (0x100 + MAX_INSTALLED_CELLS)){
    writeAddress = address & 0xFF;
    for (int i = writeAddress ; i< words+writeAddress ; i++)
    {
      value = cellvalue[i].temperature+40;
      response.add(value);
    }
  }
  else if((address >= 0x200) && address+words < (0x200 + MAX_INSTALLED_CELLS)){
    writeAddress = address & 0xFF;
    for (int i = writeAddress; i< words+writeAddress ; i++)
    {
      value = (u_int16_t)cellvalue[i].impendance*100;
      response.add(value);
    }
  }
  else if((address >= 0x300) && address+words < (0x300 + MAX_INSTALLED_CELLS)){
    writeAddress = address & 0xFF;
    for (int i = writeAddress ;i <  words+writeAddress; i++)
    {
      value = sendValue[i];
      response.add(value);
    }
  }
  else if((address >= 0x400) && address+words < (0x400 + 255)){
    writeAddress = address & 0x00FF;
    //Serial.printf("\nFunction code 03 %d[%d] %d ",address,writeAddress,words);
    sendValue[40]=now.Year();
    sendValue[41]=now.Month();
    sendValue[42]=now.Day();
    sendValue[43]=now.Hour();
    sendValue[44]=now.Minute();
    sendValue[45]=now.Second();
    sendValue[46]=2;
    sendValue[47]=20;
    sendValue[48]=150;
    sendValue[49]=1450;
    sendValue[50]=850;
    sendValue[51]=10000;
    for (int i = writeAddress; i < writeAddress+words; i++)
    {
      if( i<40)
        value = cellvalue[i].compensation;
      else if( i>=40 && i <= 255)
        value = sendValue[i];
      else 
        value = 0;
      response.add(value);
    }
  }
  else if((address >= 0x700) && address <= 0x7FF){
    writeAddress = address & 0xFF;
    for (int i = writeAddress; i < words; i++)
    {
      uint8_t _modBusID = EEPROM.readByte(1);
      value = _modBusID;
      response.add(value);
    }
  }
  return response;
};
