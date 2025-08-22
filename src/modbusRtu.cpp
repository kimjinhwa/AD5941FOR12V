#include <Arduino.h>
#include <EEPROM.h>
#include "modbusRtu.h"
#include "maingrobal.h"
#include <ModbusClientRTU.h>


char strErrorMessage[40];
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
void setSendbuffer(uint8_t fCode,uint16_t *sendValue){
  struct timeval tmv;
  gettimeofday(&tmv, NULL);
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
    //에러가 있다면 여기에 값을 적어 넣는다. 최대 30글자이다.
  }
  if(fCode== 3)
  {
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
  sendValue[120]=0;
  sendValue[121]=0;
  sendValue[122]=0;
  sendValue[123]=0;
  sendValue[124]=0;
  sendValue[125]=0;
  sendValue[126]= systemDefaultValue.modbusId ;
  sendValue[127]= systemDefaultValue.installed_cells;
  sendValue[128]= systemDefaultValue.AlarmTemperature;
  sendValue[129]= systemDefaultValue.alarmHighCellVoltage ;
  sendValue[130]=systemDefaultValue.alarmLowCellVoltage;
  sendValue[131]= systemDefaultValue.AlarmAmpere ;  // 200A
  for(int  i=132;i<160;i++) sendValue[i] =0x00;

  //setErrorMessageToModbus(true,"Hello....\n");
  char *dest ;
  dest = (char*)(sendValue+141); //strncpy(dest ,strErrorMessage,sizeof(strErrorMessage)-2);
  for(int i=0; i< 38;i++){
    dest[i] = strErrorMessage[i+2]; // Serial.printf("%02x ",dest[i]);
  }
  //sendValue[140]= ((int)strErrorMessage[0] << 8) & ((int)strErrorMessage[1] & 0x00ff) ;
  if(strErrorMessage[0] != 0 || strErrorMessage[1] != 0) sendValue[140]=1;
  // ESP_LOGI("TEST","\n-------> send Message Value %s %d %d %d",
  //   dest,sendValue[140],strErrorMessage[0],strErrorMessage[1] );
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

  if(  words ==0  ||  ((address & 0x00FF) + words) > 255){
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
      break;
    case 121:
      break;
    case 122:
      break;
    case 123:
      break;
    case 124:
      break;
    case 125:
      break;

    default:
      break;
    }

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
    //ModbusMessage rc =  syncRequestCellModule(token, moduleAddress, request.getFunctionCode(), writeAddress,  value);
  }
  return response;
};