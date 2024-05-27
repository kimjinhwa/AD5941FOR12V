/*
   Copyright (c) 2019 Stefan Kremser
   This software is licensed under the MIT License. See the license file for details.
   Source: github.com/spacehuhn/SimpleCLI
 */

#include "SimpleCLI.h"
#include <RtcDS1302.h>
#include "mainGrobal.h"
#include "batDeviceInterface.h"
#include "ModbusTypeDefs.h"
#include "EEPROM.h"
#include "ModbusServerRTU.h"

LittleFileSystem lsFile;
SimpleCLI simpleCli;
extern ModbusServerRTU LcdCell485;
extern BatDeviceInterface batDevice;
extern _cell_value cellvalue[MAX_INSTALLED_CELLS];
static char TAG[] ="CLI" ;
extern "C" {
#include "c/cmd.h"       // cmd
#include "c/parser.h"    // parse_lines
#include "c/cmd_error.h" // cmd_error_destroy
}
int makeRelayControllData(uint8_t *buf,uint8_t modbusId,uint8_t funcCode, uint16_t address, uint16_t len);
int readResponseData(uint8_t modbusId,uint8_t funcCode, uint8_t *buf,uint8_t len,uint16_t timeout);
void setRtcNewTime(RtcDateTime rtc);
void readnWriteEEProm();
RtcDateTime getDs1302GetRtcTime();
bool SelectBatteryMinusPlus(uint8_t modbusId);
void getTime(){

  RtcDateTime now;
  struct timeval tmv;
  gettimeofday(&tmv,NULL);
  //struct tm *timeinfo = gmtime(&tmv.tv_sec);
  //now =  RtcDateTime (tmv.tv_sec); //
  now =  getDs1302GetRtcTime();
  simpleCli.outputStream->printf("\nDs1302 Now Time is %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());

  //시스템의 시간도 같이 맞추어 준다.
  settimeofday(&tmv, NULL);
}

void ls_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  lsFile.listDir("/spiffs/", NULL);
}
void rm_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\r\n");

  if (argVal.length() == 0)
    return;

  lsFile.rm(argVal);
  // if (!argVal.startsWith("*"))
  // {
  //   argVal = String("/spiffs/") + argVal;
  // }
};
void df_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  lsFile.df();
}
void reboot_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  simpleCli.outputStream->println( "\r\nNow System Rebooting...\r\n");
  esp_restart();
}
void format_configCallback(cmd *cmdPtr)
{
  simpleCli.outputStream->printf("\r\nWould you system formating(Y/n)...\r\n");
  int c = 0;
  while (1)
  {
    if (simpleCli.inputStream->available())
      c = Serial.read();

    if (c == 'Y')
    {
      lsFile.littleFsInit(1);
      simpleCli.outputStream->printf("\r\nSystem format completed\r\n");
      return;
    }
    if (c == 'n')
      return;
  }
}
void cat_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\r\n");

  if (argVal.length() == 0)
    return;
  argVal = String("/spiffs/") + argVal;

  lsFile.cat(argVal);
}


uint16_t checkAlloff(uint32_t *failedBatteryNumberH,uint32_t *failedBatteryNumberL);

float AD5940_calibration(float *real , float *image);

void batnumber_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\nEEPROM installed Bat number %d", systemDefaultValue.installed_cells);
  if (argVal.length() == 0)
  {
    return;
  }
  systemDefaultValue.installed_cells = argVal.toInt();
  EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  EEPROM.commit();
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  simpleCli.outputStream->printf("\nChanged EEPROM installed Bat number %d", systemDefaultValue.installed_cells);
}

void AD5940_Main(void *parameters);
uint32_t sendGetMoubusModuleData(uint32_t token,uint8_t modbusId, uint8_t fCode,uint16_t startAddress, uint16_t len);

    // simpleCli.outputStream->printf("\ncount is %d",cmd.countArgs()); 
    // simpleCli.outputStream->printf("\nfirst arg is %s",cmd.getArgument(0).getName()); 
    // simpleCli.outputStream->printf("\nfirst arg is %s",cmd.getArgument(0).getValue()); 
    // simpleCli.outputStream->printf("\nsecond arg is %s",cmd.getArgument(1).getName()); 
    // simpleCli.outputStream->printf("\nsecond arg is %s",cmd.getArgument(1).getValue()); 
    // simpleCli.outputStream->printf("\nthird arg is %s",cmd.getArgument(2).getName()); 
    // simpleCli.outputStream->printf("\nthird arg is %s",cmd.getArgument(2).getValue()); 
    // simpleCli.outputStream->printf("\nforth arg is %s",cmd.getArgument(3).getName()); 
    // simpleCli.outputStream->printf("\nforth arg is %s",cmd.getArgument(3).getValue()); 
  // cmd_config.addFlagArgument("i");
  // cmd_config.addFlagArgument("v");
  // cmd_config.addPositionalArgument("num");
  // cmd_config.addPositionalArgument("value");
void printCompensationValue(){
  simpleCli.outputStream->printf("\r\nCompensation Value");
  simpleCli.outputStream->printf("\r\n\tno\timp\tvol\r\n");
  for (int i = 0; i < 10; i++){
    simpleCli.outputStream->printf("\t%d\t%d\t%d\r\n",i+1,systemDefaultValue.impendanceCompensation[i],systemDefaultValue.voltageCompensation[i] );
  }
  for (int i = 10; i < 20; i++){
    simpleCli.outputStream->printf("\t%d\t%d\t%d\r\n",i+1,systemDefaultValue.impendanceCompensation[i],systemDefaultValue.voltageCompensation[i] );
  }
  simpleCli.outputStream->printf("\r\n-------------------------\r\n");
}

void writeCellLog_configCallback(cmd *cmdPtr){
  simpleCli.outputStream->printf("\nwrite Cell Log");
  lsFile.writeCellDataLog();
}
void readCellLog_configCallback(cmd *cmdPtr){

  simpleCli.outputStream->printf("\nRead Cell Log");
  lsFile.readCellDataLog(0);
}
void measuredvalue_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg;
  int16_t number;
  float imp;
  float vol;
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  number = cmd.getArgument("num").getValue().toInt();
  imp = cmd.getArgument("imp").getValue().toFloat();
  vol = cmd.getArgument("vol").getValue().toFloat();
  simpleCli.outputStream->printf("\nnumber\timp\tvol\n"); 
  simpleCli.outputStream->printf("%d \t%3.3f\t %3.3f\n",number,imp,vol); 

  timeval tmv;
  gettimeofday(&tmv, NULL);
  _cell_value_iv value;
  value.CellNo = number;
  value.temperature = 25;;
  value.impendance = imp;
  value.voltage= vol;
  value.readTime = tmv.tv_sec;
  lsFile.writeMeasuredValue(value);

}
void offset_configCallback(cmd *cmdPtr)
{
  Command cmd(cmdPtr);
  Argument arg;
  int16_t number;
  int16_t value;
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  printCompensationValue();
  arg = cmd.getArgument("ia");
  if (arg.isSet())
  { // Impedance
    if (cmd.countArgs() < 4)
    {
      simpleCli.outputStream->printf("\n You have to input number and value like as \noffset 1 100");
      return;
    }
    number = cmd.getArgument("num").getValue().toInt();
    value = cmd.getArgument("value").getValue().toInt();
    simpleCli.outputStream->printf("\nNumber %d Value %d ", number, value);
    simpleCli.outputStream->printf("\n Cellnumber %d Installed cell %d\n", number, systemDefaultValue.installed_cells);
    if (number == 0)
    {
      simpleCli.outputStream->printf("\nAll Cell will be write  with value (%d)", value);
      for (int i = 0; i < systemDefaultValue.installed_cells; i++)
      {
        systemDefaultValue.impendanceCompensation[i] += value;
        cellvalue[i].impendance = cellvalue[i].impendance + systemDefaultValue.impendanceCompensation[i] / 100.f;
        simpleCli.outputStream->printf("\ncompansation %d IMP: %3.2f", systemDefaultValue.impendanceCompensation[i], cellvalue[i].impendance);
        EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
        EEPROM.commit();
        EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      }
      printCompensationValue();
      return;
    }
    if (number > systemDefaultValue.installed_cells)
    {
      simpleCli.outputStream->printf("\n Cell(%d) number Must be less then Installed cell(%d)\n", number, systemDefaultValue.installed_cells);
      return;
    }
    systemDefaultValue.impendanceCompensation[number - 1] += value;
    cellvalue[number-1].impendance = cellvalue[number-1].impendance + systemDefaultValue.impendanceCompensation[number-1] / 100.f;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\nWrite Ok to IMP EEPROM %d : %d", number, systemDefaultValue.impendanceCompensation[number - 1]);
  }

  arg = cmd.getArgument("va");
  if (arg.isSet())
  { // Voltage
    if (cmd.countArgs() < 4)
    {
      simpleCli.outputStream->printf("\n You have to input number and value like as \noffset 1 100");
      return;
    }
    number = cmd.getArgument("num").getValue().toInt();
    value = cmd.getArgument("value").getValue().toInt();
    if (number == 0)
    {
      simpleCli.outputStream->printf("\nAll Cell will be compansate to Value %d", value);
      for (int i = 0; i < systemDefaultValue.installed_cells; i++)
      {
        systemDefaultValue.voltageCompensation[i] += value; // cellvalue[0].voltage - cellvalue[i].voltage;
        simpleCli.outputStream->printf("\ncompansation %d ", systemDefaultValue.voltageCompensation[i]);
      }
      cellvalue[number-1].voltage= cellvalue[number-1].voltage+ systemDefaultValue.voltageCompensation[number-1] ;
      EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      EEPROM.commit();
      EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      printCompensationValue();
      return;
    }
    if (number > systemDefaultValue.installed_cells)
    {
      simpleCli.outputStream->printf("\n Cell(%d) number Must be less then Installed cell(%d)\n", number, systemDefaultValue.installed_cells);
      return;
    }
    systemDefaultValue.voltageCompensation[number - 1] += value;
    cellvalue[number-1].voltage= cellvalue[number-1].voltage+ systemDefaultValue.voltageCompensation[number-1] ;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\nWrite Ok to VOL EEPROM %d : %d", number, systemDefaultValue.voltageCompensation[number - 1]);
    printCompensationValue();
    return;
  }

  arg = cmd.getArgument("i");
  if (arg.isSet())
  { // Impedance
    if (cmd.countArgs() < 4)
    {
      simpleCli.outputStream->printf("\n You have to input number and value like as \noffset 1 100");
      return;
    }
    number = cmd.getArgument("num").getValue().toInt();
    value = cmd.getArgument("value").getValue().toInt();
    simpleCli.outputStream->printf("\nNumber %d Value %d ", number, value);
    simpleCli.outputStream->printf("\n Cellnumber %d Installed cell %d\n", number, systemDefaultValue.installed_cells);
    if (number == 0)
    {
      simpleCli.outputStream->printf("\nAll Cell will be write  with value (%d)", value);
      for (int i = 0; i < systemDefaultValue.installed_cells; i++)
      {
        systemDefaultValue.impendanceCompensation[i] = value;
        cellvalue[i].impendance = cellvalue[i].impendance + systemDefaultValue.impendanceCompensation[i] / 100.f;
        simpleCli.outputStream->printf("\ncompansation %d IMP: %3.2f", systemDefaultValue.impendanceCompensation[i], cellvalue[i].impendance);
        EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
        EEPROM.commit();
        EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      }
      printCompensationValue();
      return;
    }
    if (number > systemDefaultValue.installed_cells)
    {
      simpleCli.outputStream->printf("\n Cell(%d) number Must be less then Installed cell(%d)\n", number, systemDefaultValue.installed_cells);
      return;
    }
    systemDefaultValue.impendanceCompensation[number - 1] = value;
    cellvalue[number - 1].impendance = cellvalue[number - 1].impendance + systemDefaultValue.impendanceCompensation[number - 1] / 100.f;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\nWrite Ok to IMP EEPROM %d : %d", number, systemDefaultValue.impendanceCompensation[number - 1]);
    printCompensationValue();
    return;
  }
  arg = cmd.getArgument("v");
  if (arg.isSet())
  { // Voltage
    if (cmd.countArgs() < 4)
    {
      simpleCli.outputStream->printf("\n You have to input number and value like as \noffset 1 100");
      return;
    }
    number = cmd.getArgument("num").getValue().toInt();
    value = cmd.getArgument("value").getValue().toInt();
    if (number == 0)
    {
      simpleCli.outputStream->printf("\nAll Cell will be compansate to Value %d", value);
      for (int i = 0; i < systemDefaultValue.installed_cells; i++)
      {
        systemDefaultValue.voltageCompensation[i] = value; // cellvalue[0].voltage - cellvalue[i].voltage;
        cellvalue[number - 1].voltage= cellvalue[number - 1].voltage+ systemDefaultValue.voltageCompensation[number - 1];
        simpleCli.outputStream->printf("\ncompansation %d ", systemDefaultValue.voltageCompensation[i]);
      }
      EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      EEPROM.commit();
      EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
      printCompensationValue();
      return;
    }
    if (number > systemDefaultValue.installed_cells)
    {
      simpleCli.outputStream->printf("\n Cell(%d) number Must be less then Installed cell(%d)\n", number, systemDefaultValue.installed_cells);
      return;
    }
    systemDefaultValue.voltageCompensation[number - 1] = value;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    cellvalue[number - 1].voltage= cellvalue[number - 1].voltage+ systemDefaultValue.voltageCompensation[number - 1];
    simpleCli.outputStream->printf("\nWrite Ok to VOL EEPROM %d : %d", number, systemDefaultValue.voltageCompensation[number - 1]);
    printCompensationValue();
    return;
  }
  arg = cmd.getArgument("vv");
  if (arg.isSet())
  { // Voltage
    if (cmd.countArgs() < 4)
    {
      simpleCli.outputStream->printf("\n You have to input number and value like as \noffset 1 3.49 <--real Measured value");
      return;
    }
    number = cmd.getArgument("num").getValue().toInt();
    float fvalue = cmd.getArgument("value").getValue().toFloat();
    if (number == 0)
    {
      simpleCli.outputStream->printf("\nNot support value  %d", number);
      return ;
    }
    if (number > systemDefaultValue.installed_cells)
    {
      simpleCli.outputStream->printf("\n Cell(%d) number Must be less then Installed cell(%d)\n", number, systemDefaultValue.installed_cells);
      return;
    }
    //calculate 
    float gapVoltage = fvalue - cellvalue[number-1].voltage ;
    simpleCli.outputStream->printf("\nreadVolage %3.2f - inputVol %3.2f = gapVoltage %3.2f", cellvalue[number-1].voltage , fvalue,gapVoltage );
    int voloffset = (int)(gapVoltage /0.005);
    simpleCli.outputStream->printf("\ngapVoltege %3.2f/0.005 = %d",gapVoltage, voloffset);
    //이미 offset이 적용되어 있는 값이므로 +=를 해준다.
    systemDefaultValue.voltageCompensation[number - 1] = voloffset;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\nWrite Ok to VOL EEPROM %d : %d", number, systemDefaultValue.voltageCompensation[number - 1]);
    printCompensationValue();
    return;
  }

  // String strValue ;
  //   strValue = arg.getValue();
  //   simpleCli.outputStream->printf("\nTime will Set to %d/%d/%d %d:%d:%d ",strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
  //   now = RtcDateTime(strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
  //   setRtcNewTime(now);
  //   now = getDs1302GetRtcTime();
  //   simpleCli.outputStream->printf("\nNow New time is set to %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
  //   return ;
  //}
  // arg = cmd.getArgument("Month");
  // return;
}

uint16_t sendGetChangeModuleId(uint8_t modbusId, uint8_t fCode);
uint16_t sendGetModuleId(uint8_t modbusId, uint8_t fCode);
void moduleid_configCallback(cmd *cmdPtr){
  uint16_t  moduleId;
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\r\n%s",argVal.c_str());
  if(argVal.length() > 0 ){
    moduleId= sendGetChangeModuleId(argVal.toInt(), 06);
    simpleCli.outputStream->printf("\r\nModule Id Changed to: %d",moduleId);
    return;
  }
  else{
    moduleId= sendGetModuleId(255, 04);
    simpleCli.outputStream->printf("\r\nModule Id is  %d",moduleId);
    return;
  }
}
void temperature_configCallback(cmd *cmdPtr){
  uint16_t  batTemperature;
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\r\n%s",argVal.c_str());
  if(argVal.length() > 0 ){

    //sendGetMoubusModuleData(uint8_t modbusId, uint8_t fCode);
    batTemperature = sendGetMoubusModuleData(millis(), argVal.toInt(), READ_INPUT_REGISTER,0,3);
    simpleCli.outputStream->printf("\r\n[%d]Bat Temperature : %3.2f",argVal.toInt(),batTemperature/100.0f);
    return;
  }
  for (int i = 1; i <= systemDefaultValue.installed_cells; i++)
  {
    batTemperature = sendGetMoubusModuleData(millis(), i, READ_INPUT_REGISTER,0,3);
    //batTemperature = sendGetMoubusModuleData(i, 04);
    simpleCli.outputStream->printf("\r\n[%d]Bat Temperature : %3.2f",i,batTemperature/100.0f);
  }
}
void impedance_configCallback(cmd *cmdPtr){
  AD5940_Main(simpleCli.outputStream);
}
void calibration_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  simpleCli.outputStream->printf("\r\n%s",argVal.c_str());

  uint8_t relayPos;
  float real , image;
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  simpleCli.outputStream->printf("\nEEPROM Real Image IMP:%6.2f\t %6.2f\t ",
      systemDefaultValue.real_Cal,systemDefaultValue.image_Cal);

  if(systemDefaultValue.runMode != 0 ){
    simpleCli.outputStream->printf("\r\nMust run at manual mode");
    return;
  }
  simpleCli.outputStream->printf("\nNow Start calibrating...Wait...");
  float ImpMagnitude = AD5940_calibration(&real , &image);
  simpleCli.outputStream->printf("\nRcalVolt Real Image IMP:%6.2f\t %6.2f\t %6.2f (%dmills)",
    real,image,ImpMagnitude);
  if(argVal.equals("save")  ){
    systemDefaultValue.image_Cal = image;
    systemDefaultValue.real_Cal  = real;
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\nEEPROM Real Image IMP:%6.2f\t %6.2f\t ",
      systemDefaultValue.real_Cal,systemDefaultValue.image_Cal);
    simpleCli.outputStream->printf("\r\nyou must reboot system for appply");
  } 

}
void id_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  if (argVal.length() == 0){
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\r\nmode id is  %d",systemDefaultValue.modbusId);
    return;
  }
  int8_t mod_id = argVal.toInt(); 
  if(mod_id > 0 && mod_id <= 247){
    systemDefaultValue.modbusId= mod_id; 
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\r\nmodbus is Changed to %d",systemDefaultValue.modbusId);
  }
}
void mode_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg = cmd.getArgument(0);
  String argVal = arg.getValue();
  if (argVal.length() == 0){
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\r\nPlease Input Mode ");
    simpleCli.outputStream->printf("\r\n0 : manual mode  1: Voltage only auto 3: Vol & Imp Auto");
    simpleCli.outputStream->printf("\r\nCurrent mode %d",systemDefaultValue.runMode );
    simpleCli.outputStream->printf("\r\nyou must reboot system for appply");
    return;
  }
  int8_t mode = argVal.toInt(); 
  if(mode == 0 || mode == 1 || mode == 3){
    systemDefaultValue.runMode = mode; 
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
    EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    simpleCli.outputStream->printf("\r\nmode Changed %d",systemDefaultValue.runMode );
  }
}

bool sendSelectBatteryWithNoCheck(uint8_t modbusId);
void relay_configCallback(cmd *cmdPtr){
  Command cmd(cmdPtr);
  Argument arg ;
  String strValue ;
  uint8_t relayPos;
  uint8_t buf[64];
  arg = cmd.getArgument("sel");

  if (arg.isSet())
  {
      strValue = arg.getValue();
      relayPos = strValue.toInt();
      uint16_t readCount;
      if (relayPos == 0)
      {
          makeRelayControllData(buf, 0xFF, 5, 0, 0x00); // 0xff BROADCAST
          delay(100);
          makeRelayControllData(buf, 0xFF, 5, 1, 0x00); // 0xff BROADCAST
          delay(100);
          simpleCli.outputStream->printf("\nAll relay offed..");
      }
      else if (relayPos >= 1 || relayPos <= 20)
      {
          simpleCli.outputStream->printf("\nRelay %d Selecet", relayPos);
          SelectBatteryMinusPlus(relayPos);

          time_t startRead = millis();
          float batVoltage = 0.0;
          batVoltage = batDevice.readBatAdcValue(relayPos ,600);
          cellvalue[relayPos - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
          time_t endRead = millis();                    // take 300ms
          simpleCli.outputStream->printf("Bat Voltage is : %3.3f (%ldmilisecond)", batVoltage, endRead - startRead);
      }
      else
      {
          makeRelayControllData(buf, 0xFF, 5, 0, 0x00); // 0xff BROADCAST
          delay(100);
          makeRelayControllData(buf, 0xFF, 5, 1, 0x00); // 0xff BROADCAST
          delay(100);
          simpleCli.outputStream->printf("\nAll relay offed..");
      }
  }
  arg = cmd.getArgument("off");
  if(arg.isSet()){
    uint32_t failedBatteryH,failedBatteryL;
    uint16_t retValue;
  LcdCell485.suspendTask();
      retValue=checkAlloff(&failedBatteryH,&failedBatteryL);
  LcdCell485.resumeTask();
      simpleCli.outputStream->printf("retValue :0x%02x 0x%04x%04x\n",retValue,failedBatteryH,failedBatteryL);
  }
  arg = cmd.getArgument("test");
  if (arg.isSet())
  {
      strValue = arg.getValue();
      relayPos = strValue.toInt();
      uint16_t readCount;
      
      if (relayPos == 0)
      {
          makeRelayControllData(buf, 0xFF, 5, 0, 0x00); // 0xff BROADCAST
          delay(100);
          makeRelayControllData(buf, 0xFF, 5, 1, 0x00); // 0xff BROADCAST
          delay(100);
          simpleCli.outputStream->printf("\nAll relay offed..");
      }
      else if (relayPos >= 1 || relayPos <= 20)
      {
          simpleCli.outputStream->printf("\nRelay %d Selecet", relayPos);
          sendSelectBatteryWithNoCheck(relayPos);
          time_t startRead = millis();
          float batVoltage = 0.0;
          batVoltage = batDevice.readBatAdcValue(relayPos ,600);
          cellvalue[relayPos - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
          time_t endRead = millis();                    // take 300ms
          simpleCli.outputStream->printf("Bat Voltage is : %3.3f (%ldmilisecond)", batVoltage, endRead - startRead);
      }
      else
      {
          makeRelayControllData(buf, 0xFF, 5, 0, 0x00); // 0xff BROADCAST
          delay(100);
          makeRelayControllData(buf, 0xFF, 5, 1, 0x00); // 0xff BROADCAST
          delay(100);
          simpleCli.outputStream->printf("\nAll relay offed..");
      }
  }
}
void time_configCallback(cmd *cmdPtr){
  RtcDateTime now;
  Command cmd(cmdPtr);
  Argument arg ;
  String strValue ;
  // gettimeofday(&tmv,NULL);
  // struct tm *timeinfo = gmtime(&tmv.tv_sec);
  // now =  RtcDateTime (tmv.tv_sec); //
  //struct timeval tmv;
  now = getDs1302GetRtcTime();
  simpleCli.outputStream->printf("\nnow is to %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
  //tmv.tv_usec = now.TotalSeconds();

  arg = cmd.getArgument("year");
  if(arg.isSet()){
    strValue = arg.getValue();
    simpleCli.outputStream->printf("\nTime will Set to %d/%d/%d %d:%d:%d ",strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
    now = RtcDateTime(strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
    setRtcNewTime(now);
    now = getDs1302GetRtcTime();
    simpleCli.outputStream->printf("\nNow New time is set to %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
    return ;
  }
  arg = cmd.getArgument("Month");
  if(arg.isSet()){
    strValue = arg.getValue();
    now = RtcDateTime(now.Year(),strValue.toInt(),now.Day(),now.Hour(),now.Minute(),now.Second());
    setRtcNewTime(now);
    return ;
  }
  arg = cmd.getArgument("day");
  if(arg.isSet()){
    strValue = arg.getValue();
    now = RtcDateTime(now.Year(),now.Month(),strValue.toInt(),now.Hour(),now.Minute(),now.Second());
    setRtcNewTime(now);
    return ;
  }
  arg = cmd.getArgument("hour");
  if(arg.isSet()){
    strValue = arg.getValue();
    now = RtcDateTime(now.Year(),now.Month(),now.Day(),strValue.toInt(),now.Minute(),now.Second());
    setRtcNewTime(now);
    return ;
  }
  arg = cmd.getArgument("minute");
  if(arg.isSet()){
    strValue = arg.getValue();
    now = RtcDateTime(now.Year(),now.Month(),now.Day(),now.Hour(),strValue.toInt(),now.Second());
    setRtcNewTime(now);
    return ;
  }
  arg = cmd.getArgument("second");
  if(arg.isSet()){
    strValue = arg.getValue();
    return ;
  }
  getTime();
  String argVal = arg.getValue();
}
void errorCallback(cmd_error *errorPtr)
{
  CommandError e(errorPtr);

  simpleCli.outputStream->printf( (String("ERROR: ") + e.toString()).c_str());

  if (e.hasCommand())
  {
    simpleCli.outputStream->printf(  (String("Did you mean? ") + e.getCommand().toString()).c_str());
  }
  else
  {
     simpleCli.outputStream->printf(  simpleCli.toString().c_str());
  }
}
void help_Callback(cmd *cmdptr){

}
void SimpleCLI::setOutputStream(Print *outputStream ){
    this->outputStream = outputStream;
}

void SimpleCLI::setInputStream(Stream *inputStream ){
        this->inputStream = inputStream ;
}
SimpleCLI::SimpleCLI(int commandQueueSize, int errorQueueSize,Print *outputStream ) : commandQueueSize(commandQueueSize), errorQueueSize(errorQueueSize) {
  this->inputStream = &Serial;
  Command cmd_config = addCommand("ls",ls_configCallback);
  cmd_config.setDescription(" File list \r\n ");
  cmd_config =  addSingleArgCmd("cat", cat_configCallback);
  cmd_config = addSingleArgCmd("rm", rm_configCallback);
  cmd_config = addSingleArgCmd("format", format_configCallback);
  cmd_config = addCommand("time", time_configCallback);
  cmd_config.addArgument("y/ear","");
  cmd_config.addArgument("mo/nth","");
  cmd_config.addArgument("d/ay","");
  cmd_config.addArgument("h/our","");
  cmd_config.addArgument("m/inute","");
  cmd_config.addArgument("s/econd","");
  cmd_config.setDescription(" Get Time or set \r\n time -y 2024 or time -M 11,..., Month is M , minute is m ");
  cmd_config = addCommand("df", df_configCallback);
  cmd_config = addSingleArgCmd("reboot", reboot_configCallback);

  cmd_config = addCommand("r/elay", relay_configCallback);
  cmd_config.addArgument("s/el","");
  cmd_config.addArgument("t/est","");
  cmd_config.addFlagArg("off");
  cmd_config.setDescription("relay on off controll \r\n relay -s/el [1] [-off]");
  cmd_config = addSingleArgCmd("mode", mode_configCallback);
  cmd_config = addSingleArgCmd("id", id_configCallback);
  cmd_config = addSingleArgCmd("cal/ibration", calibration_configCallback);
  cmd_config = addSingleArgCmd("bat/number", batnumber_configCallback);
  cmd_config = addCommand("imp/edance", impedance_configCallback);
  cmd_config = addSingleArgCmd("temp/erature", temperature_configCallback);
  cmd_config = addSingleArgCmd("mod/uledid", moduleid_configCallback);

  cmd_config = addCommand("writecellLog",writeCellLog_configCallback);
  cmd_config = addCommand("readcellLog",readCellLog_configCallback);
  cmd_config = addCommand("wrm",measuredvalue_configCallback);
  cmd_config.addPositionalArgument("num");
  cmd_config.addPositionalArgument("imp");
  cmd_config.addPositionalArgument("vol");
  cmd_config.setDescription("\n \
    Input measured impdance and voltage compansation value\n  \
    Usage : wrm cellno imp vol \n\
  ");

  cmd_config = addCommand("offset",offset_configCallback);
  cmd_config.addFlagArgument("i"); //impedance
  cmd_config.addFlagArgument("ia"); //impedance
  cmd_config.addFlagArgument("v"); //voltage
  cmd_config.addFlagArgument("va"); //voltage
  cmd_config.addFlagArgument("vv"); //voltage
  cmd_config.addPositionalArgument("num");
  cmd_config.addPositionalArgument("value");
  cmd_config.setDescription("\nFor impdance and voltage compansation value\n  \
    Usage: offset [-i (cellnumber) (value)] [-v (cellnumber) (value)]\n    \
                [-ia][-va]  \
           flag: -ia  0 [value] [ set all offset value set to samevalue to given impedance value]\n   \
           flag: -iv  [ for set to samevalue to First voltage value]\n     \
           For set IMP input, value is 100 times.\n, \
           For set Vol input, value is for use offset. So not multiples.\n, \
             ");
  simpleCli.setOnError(errorCallback);
  cmd_config= addCommand("help",help_Callback);
}

SimpleCLI::~SimpleCLI() {
    cmd_destroy_rec(cmdList);
    cmd_destroy_rec(cmdQueue);
    cmd_error_destroy_rec(errorQueue);
}

void SimpleCLI::pause() {
    pauseParsing = true;
}

void SimpleCLI::unpause() {
    pauseParsing = false;

    // Go through queued errors
    while (onError && errored()) {
        onError(getError().getPtr());
    }

    // Go through queued commands
    if(available()) {
        cmd* prev = NULL;
        cmd* current = cmdQueue;
        cmd* next = NULL;

        while(current) {
            next = current->next;

            // Has callback, then run it and remove from queue
            if(current->callback) {
                current->callback(current);
                if(prev) prev->next = next;
                cmd_destroy(current);
            } else {
                prev = current;
            }

            current = next;
        }
    }
}

void SimpleCLI::parse(const String& input) {
    parse(input.c_str(), input.length());
}

void SimpleCLI::parse(const char* input) {
    if (input) parse(input, strlen(input));
}

void SimpleCLI::parse(const char* str, size_t len) {
    // Split str into a list of lines
    line_list* l = parse_lines(str, len);

    // Go through all lines and try to find a matching command
    line_node* n = l->first;

    while (n) {
        cmd* h       = cmdList;
        bool success = false;
        bool errored = false;

        while (h && !success && !errored) {
            cmd_error* e = cmd_parse(h, n);

            // When parsing was successful
            if (e->mode == CMD_PARSE_SUCCESS) {
                if (h->callback && !pauseParsing) h->callback(h);
                else cmdQueue = cmd_push(cmdQueue, cmd_copy(h), commandQueueSize);

                success = true;
            }

            // When command name matches but something else went wrong, exit with error
            else if (e->mode > CMD_NOT_FOUND) {
                if (onError && !pauseParsing) {
                    onError(e);
                } else {
                    errorQueue = cmd_error_push(errorQueue, cmd_error_copy(e), errorQueueSize);
                }

                errored = true;
            }

            // When command name does not match

            /*else (e->mode <= CMD_NOT_FOUND) {

               }*/

            cmd_error_destroy(e);

            cmd_reset_cli(h);

            h = h->next;
        }

        // No error but no success either => Command could not be found
        if (!errored && !success) {
            cmd_error* e = cmd_error_create_not_found(NULL, n->words->first);

            if (onError && !pauseParsing) {
                onError(e);
            } else {
                errorQueue = cmd_error_push(errorQueue, cmd_error_copy(e), errorQueueSize);
            }

            cmd_error_destroy(e);

            errored = true;
        }

        n = n->next;
    }

    line_list_destroy(l);
}

bool SimpleCLI::available() const {
    return cmdQueue;
}

bool SimpleCLI::errored() const {
    return errorQueue;
}

bool SimpleCLI::paused() const {
    return pauseParsing;
}

int SimpleCLI::countCmdQueue() const {
    cmd* h = cmdQueue;
    int  i = 0;

    while (h) {
        h = h->next;
        ++i;
    }

    return i;
}

int SimpleCLI::countErrorQueue() const {
    cmd_error* h = errorQueue;
    int i        = 0;

    while (h) {
        h = h->next;
        ++i;
    }

    return i;
}

Command SimpleCLI::getCmd() {
    if (!cmdQueue) return Command();

    // "Pop" cmd pointer from queue
    cmd* ptr = cmdQueue;

    cmdQueue = cmdQueue->next;

    // Create wrapper class and copy cmd
    Command c(ptr, COMMAND_TEMPORARY);

    // Destroy old cmd from queue
    cmd_destroy(ptr);

    return c;
}

Command SimpleCLI::getCmd(String name) {
    return getCmd(name.c_str());
}

Command SimpleCLI::getCmd(const char* name) {
    if (name) {
        cmd* h = cmdList;

        while (h) {
            if (cmd_name_equals(h, name, strlen(name), h->case_sensetive) == CMD_NAME_EQUALS) return Command(h);
            h = h->next;
        }
    }
    return Command();
}

Command SimpleCLI::getCommand() {
    return getCmd();
}

Command SimpleCLI::getCommand(String name) {
    return getCmd(name);
}

Command SimpleCLI::getCommand(const char* name) {
    return getCmd(name);
}

CommandError SimpleCLI::getError() {
    if (!errorQueue) return CommandError();

    // "Pop" cmd_error pointer from queue
    cmd_error* ptr = errorQueue;
    errorQueue = errorQueue->next;

    // Create wrapper class and copy cmd_error
    CommandError e(ptr, COMMAND_ERROR_TEMPORARY);

    // Destroy old cmd_error from queue
    cmd_error_destroy(ptr);

    return e;
}

void SimpleCLI::addCmd(Command& c) {
    if (!cmdList) {
        cmdList = c.cmdPointer;
    } else {
        cmd* h = cmdList;

        while (h->next) h = h->next;
        h->next = c.cmdPointer;
    }

    c.setCaseSensetive(caseSensetive);
    c.persistent = true;
}

Command SimpleCLI::addCmd(const char* name, void (* callback)(cmd* c)) {
    Command c(cmd_create_default(name));

    c.setCallback(callback);
    addCmd(c);

    return c;
}

Command SimpleCLI::addBoundlessCmd(const char* name, void (* callback)(cmd* c)) {
    Command c(cmd_create_boundless(name));

    c.setCallback(callback);
    addCmd(c);

    return c;
}

Command SimpleCLI::addSingleArgCmd(const char* name, void (* callback)(cmd* c)) {
    Command c(cmd_create_single(name));

    c.setCallback(callback);
    addCmd(c);

    return c;
}
Command SimpleCLI::addCommand(const char* name, void (* callback)(cmd* c)) {
    return addCmd(name, callback);
}

Command SimpleCLI::addBoundlessCommand(const char* name, void (* callback)(cmd* c)) {
    return addBoundlessCmd(name, callback);
}

Command SimpleCLI::addSingleArgumentCommand(const char* name, void (* callback)(cmd* c)) {
    return addSingleArgCmd(name, callback);
}

String SimpleCLI::toString(bool descriptions) const {
    String s;

    toString(s, descriptions);
    return s;
}

void SimpleCLI::toString(String& s, bool descriptions) const {
    cmd* h = cmdList;

    while (h) {
        Command(h).toString(s, descriptions);
        if (descriptions) s += "\r\n";
        s += "\r\n";
        h  = h->next;
    }
}

void SimpleCLI::setCaseSensetive(bool caseSensetive) {
    this->caseSensetive = caseSensetive;

    cmd* h = cmdList;

    while (h) {
        h->case_sensetive = caseSensetive;
        h                 = h->next;
    }
}

void SimpleCLI::setCaseSensitive(bool caseSensitive) {
    setCaseSensetive(caseSensitive);
}

void SimpleCLI::setOnError(void (* onError)(cmd_error* e)) {
    this->onError = onError;
}

void SimpleCLI::setErrorCallback(void (* onError)(cmd_error* e)) {
    setOnError(onError);
}