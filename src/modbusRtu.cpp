#include <Arduino.h>
#include <EEPROM.h>
#include "modbusRtu.h"
#include "maingrobal.h"
#include <ModbusClientRTU.h>
#include "mainClass.hpp"

extern uint8_t isAD5940ReInit;
char strErrorMessage[40];
static int saveToBaseVoltage;
void setMeasureImpendanceNow(bool value);
bool getMeasureImpendanceNow();

void setErrorMessageToModbus(bool setError,const char* msg)
{
  memset(strErrorMessage,0x00,sizeof(strErrorMessage));
  if(setError){
    strErrorMessage[0]=0;
    strErrorMessage[1]=setError;
    strncpy(strErrorMessage + 2, msg, sizeof(strErrorMessage) - 3);
    strErrorMessage[strlen(msg)+2] = '\0';
    //ESP_LOGI("MODBUS","--->StrLen is %d",strlen(msg));
    strErrorMessage[sizeof(strErrorMessage) - 1] = '\0';
  }
  else 
  {
    strErrorMessage[0]=0;
    strErrorMessage[1]=0;
  }
};
extern float maxTemperature;

void setSendbuffer(uint8_t fCode,uint16_t *sendValue){
  struct timeval tmv;
  gettimeofday(&tmv, NULL);
  struct tm *timeinfo = gmtime(&tmv.tv_sec);
  //String strLog = getTimeString(tmv.tv_sec);
  if(fCode== 4){
    for(int i=0;i<20;i++){
      sendValue[i] = (uint16_t)(cellvalue[i].voltage *100);
      sendValue[i+20] = (uint16_t)(systemDefaultValue.baseVoltage[i]);
    }
    int16_t temperature; 
    for(int i=40;i<80;i++){
      sendValue[i] = cellvalue[i-40].temperature ;
      //*(sendValue+i) = (uint16_t)();
    }
    for(int i=80;i<100;i++){
      sendValue[i] = (uint16_t)(cellvalue[i-80].impendance*100);
      sendValue[i+20] = (uint16_t)(systemDefaultValue.baseImpendance[i-80]);
    }
    //에러가 있다면 여기에 값을 적어 넣는다. 최대 30글자이다.
  }
  if(fCode== 3)
  {
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    for(int i=0;i<20;i++){
      sendValue[i] = (uint16_t)(systemDefaultValue.voltageCompensation[i]);
      sendValue[i+20] = (uint16_t)(systemDefaultValue.baseVoltage[i]);
    }
    int16_t temperature; 
    for(int i=40;i<80;i++){
      sendValue[i] = (uint16_t)(maxTemperature);
      //*(sendValue+i) = (uint16_t)();
    }
    for(int i=80;i<100;i++){
      sendValue[i] = (uint16_t)(systemDefaultValue.impendanceCompensation[i-80]);
      sendValue[i+20] = (uint16_t)(systemDefaultValue.baseImpendance[i-80]);
    }

  }
  /*timesetting */
  sendValue[120]=  timeinfo->tm_year +1900;
  sendValue[121]=  timeinfo->tm_mon+1;
  sendValue[122]=  timeinfo->tm_mday;
  sendValue[123]=  timeinfo->tm_hour;
  sendValue[124]=  timeinfo->tm_min;
  sendValue[125]=  timeinfo->tm_sec;
  /*--------------------------------*/  
  sendValue[126]= systemDefaultValue.modbusId ;
  sendValue[127]= systemDefaultValue.installed_cells;
  sendValue[128]= systemDefaultValue.AlarmTemperature;
  sendValue[129]= systemDefaultValue.alarmHighCellVoltage ;
  sendValue[130]=systemDefaultValue.alarmLowCellVoltage;
  sendValue[131]= systemDefaultValue.AlarmAmpere ;  // 200A
  //for(int  i=132;i<160;i++) sendValue[i] =0x00;
  sendValue[132] = systemDefaultValue.ImpedanceFactor;
  sendValue[133] = systemDefaultValue.VoltageFactor;
  sendValue[134] = systemDefaultValue.TemperatureFactor;
  sendValue[135] = systemDefaultValue.runMode;
  sendValue[136] = systemDefaultValue.logLevel;
  sendValue[137]=systemDefaultValue.real_Cal;
  sendValue[138]=systemDefaultValue.image_Cal;
  sendValue[139]=systemDefaultValue.ImpedanceMeasurePeriod;


  sendValue[141] = systemDefaultValue.ACVoltPP;
  sendValue[142] = systemDefaultValue.DCVolt;
  sendValue[143] = systemDefaultValue.SinFreq;
  sendValue[144] = systemDefaultValue.RcalLoopCount;
  sendValue[145] = selectCell.getCurrentPort();
  sendValue[146] = getMeasureImpendanceNow();
  sendValue[147] = saveToBaseVoltage;
}
char modbusCellData[100];

ModbusMessage FC03(ModbusMessage request) 
{
  uint16_t address;           // requested register address
  uint16_t writeAddress;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back
  uint16_t value;
  uint16_t sendValue[256];

  struct timeval tmv;
  gettimeofday(&tmv, NULL);
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

    for (int i = 0; i < words; i++)
    {
      response.add(pValues[i]);
    }
  }
  else if(address >= 0x50010  ){  //  5941제어이다.
  }

  return response;
};
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

  if(  words ==0  ||  words > 256){
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  } 
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  if ((address + words) < 0x100)// 256보다 작으면...
  {
    //Serial.printf("\nFunction code 04 %d[%d] %d ",address,writeAddress,words);
      for (i = address; i < words + address; i++)
      {
        value = sendValue[i];
        response.add(value);
        //Serial.printf(" %d",value);
      }
  }
  else if((address >= 0x100) && address+words < (0x100 + 40)){
    for (i = writeAddress ; i< words+writeAddress ; i++)
    {
        value = sendValue[i+40];
        response.add(value);
    }
  }
  else if((address >= 0x200) && address+words < (0x200 + 40)){
    writeAddress = address & 0x00FF; //중복코드이다 이미 마스킹되어 있다.
    for (i = writeAddress; i< words+writeAddress ; i++)
    {
        value = sendValue[i+80];
        response.add(value);
    }
  }
  else if((address >= 0x300) && address+words < (0x300 + 40)){
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
    uint8_t moduleAddress = address >> 8;//-30000;
    moduleAddress  -= 16;
    //moduleAddress = (int16_t)(moduleAddress / 3) +1;
    uint16_t *pValues;
    pValues= (uint16_t *)&modbusCellData;
    words = words > 16 ? 16 :words;
    uint32_t token=millis();
    uint32_t restoken=millis();
    //TODO: 
    //   1, request.getFunctionCode(), 0,  3);

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
  // uint16_t quantity;
  // // get request quantitys
  // request.get(2, address);
  // request.get(4, quantity);
  // uint16_t writeAddress = (0xFFFF & address);

  // response.add(request.getServerID(), request.getFunctionCode());
  // ESP_LOGI("MODBUS", "\nFunction code %d address(%d) writeAddress(%d) quantity(%d) ",
  //   response.getFunctionCode(), address, writeAddress, quantity);

  // if (writeAddress >= 0x1101 && writeAddress <= 0x2501)
  // { // Cell제어
  //   uint8_t moduleAddress = address >> 8;
  //   moduleAddress -= 16;
  //   writeAddress &= 0x00FF;
  //   writeAddress = writeAddress - 1;
  //   uint32_t token = millis();
  //   ModbusMessage rc = syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress, quantity);

  //   std::vector<uint8_t> MM_data(rc.data(), rc.data() + rc.size());
  //   for (uint8_t byte : MM_data)
  //   {
  //     ESP_LOGI("MODSERVER", "%d", byte);
  //   }
  //   uint8_t relay =static_cast<uint8_t >(MM_data[3]);
  //   response.add((uint8_t)1);
  //   response.add(relay);
  //   ESP_LOGI("REQ", "server id (%d) func %d ", request.getServerID(), request.getFunctionCode());
  //   ESP_LOGI("REQ", "server id (%d) func %d error %d", rc.getServerID(), rc.getFunctionCode(), rc.getError());
  // }
  return response;
};
ModbusMessage FC05(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t value;
  
  request.get(2, address);
  request.get(4, value);
  uint16_t writeAddress = (0xFFFF & address);

  struct timeval tmv;
  gettimeofday(&tmv, NULL);

  response.add(request.getServerID(), request.getFunctionCode(), writeAddress);
  response.add(value);

  ESP_LOGI("MODBUS", "\nFunction code %d address(%d) writeAddress(%d) value(%d) ",
    response.getFunctionCode(), address, writeAddress, value);

  if(writeAddress >= 0x1101 && writeAddress <= 0x2501  ){  // Cell제어 
    uint8_t moduleAddress = address >> 8;
    moduleAddress  -= 16;
    writeAddress &= 0x00FF;
    writeAddress = writeAddress -1;
    uint32_t token=millis();
    //ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress,  value);
  }
  return response;
};

bool isAD5940StructInit_valueChanged = false;
void setSelectCell(uint8_t cellNumber);
ModbusMessage FC06(ModbusMessage request)
{
  uint16_t address;       // requested register address
  ModbusMessage response; // response message to be sent back
  uint16_t value;
  // get request values
  request.get(2, address);
  request.get(4, value);
  uint16_t writeAddress = (0xFFFF & address);

  struct timeval tmv;
  gettimeofday(&tmv, NULL);

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
  if (writeAddress < 20)  // voltage compensation
  {
    systemDefaultValue.voltageCompensation[writeAddress] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  else if (writeAddress >= 20 && writeAddress < 40)  // voltage compensation
  {
    systemDefaultValue.baseVoltage[writeAddress - 20] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  if (writeAddress >= 40 && writeAddress < 80) // temperature
  {
  }
  if (writeAddress >= 80 && writeAddress < 100)  //impedance compensation
  {
    systemDefaultValue.impendanceCompensation[writeAddress - 80] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  if (writeAddress >= 100 && writeAddress < 120)  //impedance compensation
  {
    systemDefaultValue.baseImpendance[writeAddress - 100] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  }
  if (writeAddress >= 120 && writeAddress < 126)
  { // 시간을 설정한다.

    gettimeofday(&tmv, NULL);
    struct tm *timeinfo = gmtime(&tmv.tv_sec);
    switch (writeAddress)
    {
    case 120:
      if(value > 1900) value = value - 1900;
      timeinfo->tm_year = value;
      break;
    case 121:
      value = value - 1;
      timeinfo->tm_mon = value;
      break;
    case 122:
      timeinfo->tm_mday = value;
      break;
    case 123:
      timeinfo->tm_hour = value;
      break;
    case 124:
      timeinfo->tm_min = value;
      break;
    case 125:
      timeinfo->tm_sec = value;
      break;

    default:
      break;
    }
    tmv.tv_sec = mktime(timeinfo);
    settimeofday(&tmv, NULL);

  }
  if (writeAddress >= 126 && writeAddress < 147)
  {

    switch (writeAddress)
    {
    case 126:
      systemDefaultValue.modbusId = value;
      for(int i=0;i<20;i++){
        if(systemDefaultValue.modbusId == 1){
          systemDefaultValue.baseVoltage[i] = measuredVoltage_1[i];
          systemDefaultValue.baseImpendance[i] = measuredImpedance_1[i];
        }
        else
        {
          systemDefaultValue.baseVoltage[i] = measuredVoltage_2[i];
          systemDefaultValue.baseImpendance[i] = measuredImpedance_2[i];
        }
      }
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
      isAD5940StructInit_valueChanged = true;
      systemDefaultValue.AlarmAmpere = value;
      break;
    case 132:
      systemDefaultValue.ImpedanceFactor = value;
      break;
    case 133:
      systemDefaultValue.VoltageFactor = value;
      break;
    case 134:
      systemDefaultValue.TemperatureFactor = value;
      break;
    case 135:
      systemDefaultValue.runMode = value;
      break;
    case 136:
      systemDefaultValue.logLevel = value;
      break;
    case 137:
      systemDefaultValue.real_Cal = value;
      break;
    case 138:
      systemDefaultValue.image_Cal = value;
      break;
    case 139:
      systemDefaultValue.ImpedanceMeasurePeriod = value;
      break;
    case 141:
      isAD5940StructInit_valueChanged = true;
      systemDefaultValue.ACVoltPP = value;
      break;
    case 142:
      isAD5940StructInit_valueChanged = true;
      systemDefaultValue.DCVolt = value;
      break;
    case 143:
      isAD5940StructInit_valueChanged = true;
      systemDefaultValue.SinFreq = value;
      break;
    case 144:
      isAD5940StructInit_valueChanged = true;
      systemDefaultValue.RcalLoopCount = value;
      break;
    case 145:
      selectCell.select(value);
      setSelectCell(value);
      break;
    case 146:  
      value = value >= 1 ? 1 : 0;
      setMeasureImpendanceNow(value);
      break;
    case 147:
      if( value == 1){
        for(int i=0;i<20;i++){
          systemDefaultValue.baseVoltage[i] = (uint16_t)(cellvalue[i].voltage *100);
        }
        saveToBaseVoltage = 2;
      }
      break;
    default:
      break;
    };
    if(isAD5940StructInit_valueChanged)
    {
      isAD5940ReInit = 1;
      isAD5940StructInit_valueChanged = false;
      //esp_restart();
    }
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
    //ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress,  value);
  }
  return response;
};