#include <Arduino.h>
#include <EEPROM.h>
#include "modbusRtu.h"
#include "mainGrobal.h"
#include <RtcDS1302.h>
#include "modbusCellModule.h"
#include <ModbusClientRTU.h>

extern ModbusClientRTU modBusRtuCellModule;
void setRtcNewTime(RtcDateTime rtc);
ModbusMessage  syncRequestCellModule(uint32_t token,uint8_t modbusId, uint8_t fCode,uint16_t startAddress, uint16_t len);

void setSendbuffer(uint8_t fCode,uint16_t *sendValue){
  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  RtcDateTime now;
  now = RtcDateTime(tmv.tv_sec);
  if(fCode== 4){
    for(int i=0;i<40;i++){
      sendValue[i] = (uint16_t)(cellvalue[i].voltage *100);
    }
    int16_t temperature; 
    for(int i=40;i<80;i++){
      sendValue[i] = cellvalue[i-40].temperature ;
      //*(sendValue+i) = (uint16_t)();
    }
    for(int i=80;i<120;i++){
      sendValue[i] = (uint16_t)(cellvalue[i-80].impendance*100);
    }
  }
  if(fCode== 3){
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    for(int i=0;i<40;i++){
      sendValue[i] = (uint16_t)(systemDefaultValue.voltageCompensation[i]);
    }
    int16_t temperature; 
    for(int i=40;i<80;i++){
      sendValue[i] = 0;
      //*(sendValue+i) = (uint16_t)();
    }
    for(int i=80;i<120;i++){
      sendValue[i] = (uint16_t)(systemDefaultValue.impendanceCompensation[i-80]);
    }
  }
  sendValue[120]=now.Year();
  sendValue[121]=now.Month();
  sendValue[122]=now.Day();
  sendValue[123]=now.Hour();
  sendValue[124]=now.Minute();
  sendValue[125]=now.Second();
  sendValue[126]= systemDefaultValue.modbusId ;
  sendValue[127]= systemDefaultValue.installed_cells;
  sendValue[128]= systemDefaultValue.AlarmTemperature;
  sendValue[129]= systemDefaultValue.alarmHighCellVoltage ;
  sendValue[130]=systemDefaultValue.alarmLowCellVoltage;
  sendValue[131]= systemDefaultValue.AlarmAmpere ;  // 200A
}

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
  setSendbuffer(03,sendValue);
  // get request values
  request.get(2, address);
  request.get(4, words);

  if(  words ==0  ||  ((address & 0x00FF) + words) > 255){
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  } 
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  if ((address + words) < 0x100)
  {
    for (int i = address; i < words + address; i++)
    {
        value = sendValue[i];
        response.add(value);
    }
  }
  
  if(address >= 0x1101 && address <= 0x2501  ){  // Cell제어 
    uint8_t moduleAddress = address >> 8;
    moduleAddress  -= 16;
    uint16_t *pValues;
    pValues= (uint16_t *)&modbusCellData;
    words = words > 16 ? 16 :words;
    uint32_t token=millis();
    uint32_t restoken=millis();
    ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), 0,  3);
    ESP_LOGI("REQ","server id (%d) func %d ",request.getServerID(),request.getFunctionCode());
    ESP_LOGI("REQ","server id (%d) func %d error %d",rc.getServerID(),rc.getFunctionCode(),rc.getError());

    for (int i = 0; i < words; i++)
    {
      response.add(pValues[i]);
    }
  }
  else if(address >= 0x50010  ){  //  5941제어이다.
  }

  return response;
};
uint32_t getRequest_response();
ModbusMessage FC04(ModbusMessage request) {
  uint16_t address;           // requested register address
  uint16_t writeAddress;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];
  int i;
  memset(sendValue,0x00,256);
  setSendbuffer(04,sendValue);
  // get request values
  request.get(2, address);
  request.get(4, words);
  writeAddress = address & 0x00FF;

  if(  words ==0  ||  ((address & 0x00FF) + words) > 255){
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  } 
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  if ((address + words) < 0x100)
  {
    //Serial.printf("\nFunction code 04 %d[%d] %d ",address,writeAddress,words);
      for (i = address; i < words + address; i++)
      {
        value = sendValue[i];
        response.add(value);
        //Serial.printf(" %d",value);
      }
  }
  else if((address >= 0x100) && address+words < (0x100 + MAX_INSTALLED_CELLS)){
    for (i = writeAddress ; i< words+writeAddress ; i++)
    {
        value = sendValue[i+40];
        response.add(value);
    }
  }
  else if((address >= 0x200) && address+words < (0x200 + MAX_INSTALLED_CELLS)){
    writeAddress = address & 0x00FF;
    for (i = writeAddress; i< words+writeAddress ; i++)
    {
        value = sendValue[i+80];
        response.add(value);
    }
  }
  else if((address >= 0x300) && address+words < (0x300 + MAX_INSTALLED_CELLS)){
    writeAddress = address & 0x00FF;
    for (i = writeAddress ;i <  words+writeAddress; i++)
    {
      value = sendValue[i];
      response.add(value);
    }
  }
  else if((address >= 0x400) && address+words < (0x400 + 255)){
    writeAddress = address & 0x00FF;
    //Serial.printf("\nFunction code 03 %d[%d] %d ",address,writeAddress,words);
    for (i = writeAddress; i < writeAddress+words; i++)
    {
      value = sendValue[i+120];
      response.add(value);
    }
  }
  else if((address >= 0x700) && address <= 0x7FF){
    writeAddress = address & 0x00FF;
    for (i = writeAddress; i < words; i++)
    {
      uint8_t _modBusID = EEPROM.readByte(1);
      value = _modBusID;
      response.add(value);
    }
  }
  else if(address >= 0x1101 && address <= 0x2501  ){  // Cell제어 
      // response.add(modbusCellData.temperature);
      // response.add(modbusCellData.modbusid);
      // response.add(modbusCellData.baudrate);
    uint8_t moduleAddress = address >> 8;//-30000;
    moduleAddress  -= 16;
    //moduleAddress = (int16_t)(moduleAddress / 3) +1;
    uint16_t *pValues;
    pValues= (uint16_t *)&modbusCellData;
    words = words > 16 ? 16 :words;
    uint32_t token=millis();
    uint32_t restoken=millis();
    //TODO: 
    //modBusRtuCellModule
    ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), 0,  3);
    // ModbusMessage rc  = modBusRtuCellModule.syncRequest(token, 
    //   1, request.getFunctionCode(), 0,  3);
    ESP_LOGI("REQ","server id (%d) func %d ",request.getServerID(),request.getFunctionCode());
    ESP_LOGI("REQ","server id (%d) func %d error %d",rc.getServerID(),rc.getFunctionCode(),rc.getError());

    // ESP_LOGI("REQ","Request id =%d Token=%d",moduleAddress,token);
    // modbusRequestModule.addToQueue(token, moduleAddress , READ_INPUT_REGISTER, 0, 3);
    // restoken=getRequest_response();
    // for(i=0;token !=  restoken;i++){
    //   vTaskDelay(80);
    //   restoken=getRequest_response();
    //   if(i>100)break;
    // };
    // ESP_LOGI("REQ","Waiting(%d) tocken %d receive %d",i,token,restoken);

    for (i = 0; i < words; i++)
    {
      response.add(pValues[i]);
    }
  }
  else if(address >= 0x50010  ){  //  5941제어이다.
  }
  return response;
};

ModbusMessage FC01(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t quantity;
  // get request quantitys
  request.get(2, address);
  request.get(4, quantity);
  uint16_t writeAddress = (0xFFFF & address);

  response.add(request.getServerID(), request.getFunctionCode());
  ESP_LOGI("MODBUS", "\nFunction code %d address(%d) writeAddress(%d) quantity(%d) ",
    response.getFunctionCode(), address, writeAddress, quantity);

  if (writeAddress >= 0x1101 && writeAddress <= 0x2501)
  { // Cell제어
    uint8_t moduleAddress = address >> 8;
    moduleAddress -= 16;
    writeAddress &= 0x00FF;
    writeAddress = writeAddress - 1;
    uint32_t token = millis();
    ModbusMessage rc = syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress, quantity);

    std::vector<uint8_t> MM_data(rc.data(), rc.data() + rc.size());
    for (uint8_t byte : MM_data)
    {
      ESP_LOGI("MODSERVER", "%d", byte);
    }
    uint8_t relay =static_cast<uint8_t >(MM_data[3]);
    response.add((uint8_t)1);
    response.add(relay);
    ESP_LOGI("REQ", "server id (%d) func %d ", request.getServerID(), request.getFunctionCode());
    ESP_LOGI("REQ", "server id (%d) func %d error %d", rc.getServerID(), rc.getFunctionCode(), rc.getError());
    response.setError()
  }
  return response;
};
ModbusMessage FC05(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];
  memset(sendValue, 0x00, 256);
  setSendbuffer(03, sendValue);
  // get request values
  request.get(2, address);
  request.get(4, value);
  uint16_t writeAddress = (0xFFFF & address);

  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  RtcDateTime now;
  now = RtcDateTime(tmv.tv_sec);

  response.add(request.getServerID(), request.getFunctionCode(), writeAddress);
  response.add(value);

  ESP_LOGI("MODBUS", "\nFunction code %d address(%d) writeAddress(%d) value(%d) ",
    response.getFunctionCode(), address, writeAddress, value);
  ESP_LOGI("MODBUS", "Write and read %d ", systemDefaultValue.voltageCompensation[writeAddress]);

  if(writeAddress >= 0x1101 && writeAddress <= 0x2501  ){  // Cell제어 
    uint8_t moduleAddress = address >> 8;
    moduleAddress  -= 16;
    writeAddress &= 0x00FF;
    writeAddress = writeAddress -1;
    uint32_t token=millis();
    ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress,  value);
    ESP_LOGI("REQ","server id (%d) func %d ",request.getServerID(),request.getFunctionCode());
    ESP_LOGI("REQ","server id (%d) func %d error %d",rc.getServerID(),rc.getFunctionCode(),rc.getError());
  }
  return response;
};
ModbusMessage FC06(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];
  memset(sendValue, 0x00, 256);
  setSendbuffer(03, sendValue);
  // get request values
  request.get(2, address);
  request.get(4, value);
  uint16_t writeAddress = (0xFFFF & address);

  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  RtcDateTime now;
  now = RtcDateTime(tmv.tv_sec);

  // if (writeAddress > 255)
  // {
  //   response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  //   return response;
  // }
  response.add(request.getServerID(), request.getFunctionCode(), writeAddress) ;
  response.add(value);

  ESP_LOGI("MODBUS", "\nFunction code %d address(%d) writeAddress(%d) value(%d) ",
    response.getFunctionCode(), address, writeAddress, value);
  ESP_LOGI("MODBUS", "Write and read %d ", systemDefaultValue.voltageCompensation[writeAddress]);
  if (writeAddress < 40)  // voltage compensation
  {
    systemDefaultValue.voltageCompensation[writeAddress] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  if (writeAddress >= 40 && writeAddress < 80) // temperature
  {
  }
  if (writeAddress >= 80 && writeAddress < 120)  //impedance compensation
  {
    systemDefaultValue.impendanceCompensation[writeAddress - 80] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  if (writeAddress >= 120 && writeAddress < 126)
  { // 시간을 설정한다.
    switch (writeAddress)
    {
    case 120:
      now = RtcDateTime(value, now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second());
      break;
    case 121:
      now = RtcDateTime(now.Year(), value, now.Day(), now.Hour(), now.Minute(), now.Second());
      break;
    case 122:
      now = RtcDateTime(now.Year(), now.Month(), value, now.Hour(), now.Minute(), now.Second());
      break;
    case 123:
      now = RtcDateTime(now.Year(), now.Month(), now.Day(), value, now.Minute(), now.Second());
      break;
    case 124:
      now = RtcDateTime(now.Year(), now.Month(), now.Day(), now.Hour(), value, now.Second());
      break;
    case 125:
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
  if (writeAddress >= 126 && writeAddress < 132)
  {

    switch (writeAddress)
    {
    case 126:
      systemDefaultValue.modbusId = value;
      break;
    case 127:
      systemDefaultValue.installed_cells= value;
      break;
    case 128:
      systemDefaultValue.AlarmTemperature = value;
      break;
    case 129:
      systemDefaultValue.alarmHighCellVoltage = value;
      break;
    case 130:
      systemDefaultValue.alarmLowCellVoltage = value;
      break;
    case 131:
      systemDefaultValue.AlarmAmpere = value;
      break;
    default:
      break;
    };
    ESP_LOGI("MODUBS", "Write EEPROM");
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }

  if(writeAddress >= 0x1101 && writeAddress <= 0x2501  ){  // Cell제어 
    uint8_t moduleAddress = address >> 8;
    moduleAddress  -= 16;
    writeAddress &= 0x00FF;
    writeAddress = writeAddress -1;
    uint32_t token=millis();
    ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress,  value);
    ESP_LOGI("REQ","server id (%d) func %d ",request.getServerID(),request.getFunctionCode());
    ESP_LOGI("REQ","server id (%d) func %d error %d",rc.getServerID(),rc.getFunctionCode(),rc.getError());
  }
  return response;
};