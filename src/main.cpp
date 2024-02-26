#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <wifi.h>
#include <driver/adc.h>
#include "filesystem.h"
#include "mainGrobal.h"

#include "stdio.h"
//#include "ADuCM3029.h"
#include "AD5940.h"
//#include "HardwareSerialExtention.h"
#include "HardwareSerial.h"
#include <BluetoothSerial.h>
#include "myBlueTooth.h"
#include "NetworkTask.h"
#include <RtcDS1302.h>
#include "ModbusClientRTU.h"
#include "modbusRtu.h"
#include "batDeviceInterface.h"
// 기본 vSPI와 일치한다
#define VSPI_MISO   MISO  // IO19
#define VSPI_MOSI   MOSI  // IO 23
#define VSPI_SCLK   SCK   // IO 18
#define VSPI_SS     15    // IO 15

// //SPIClass * vspi = NULL;

//SPIClass SPI;
static const int spiClk = 1000000; // 1 MHz
static char TAG[] ="Main";
TaskHandle_t *h_pxNetworkTask;
TaskHandle_t *h_pxblueToothTask;
nvsSystemSet systemDefaultValue;
ThreeWire myWire(13, 14, 33); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
ModbusServerRTU rtu485(2000,CELL485_DE);// LCD를 위하여 사용한다.
ModbusServerRTU extrtu485(2000,EXT_485EN_1);
//ModbusClientRTU cellModbus(CELL485_DE);
uint8_t selecectedCellNumber =0;

_cell_value cellvalue[MAX_INSTALLED_CELLS];

void pinsetup()
{
    pinMode(READ_BATVOL, INPUT);
    // pinMode(SCK, OUTPUT);
    // pinMode(MISO, OUTPUT);
    // pinMode(MOSI, OUTPUT);

    pinMode(AD5940_ISR, OUTPUT);
    pinMode(SERIAL_SEL_ADDR0, OUTPUT);
    pinMode(SERIAL_SEL_ADDR1, OUTPUT);
    pinMode(SERIAL_TX2 , OUTPUT);
    pinMode(LED_OP, OUTPUT);

    pinMode(RTC1305_EN, OUTPUT);
    pinMode(AD636_SEL, OUTPUT);
    pinMode(CS_5940, OUTPUT);
    pinMode(EXT_485EN_1, OUTPUT);
    pinMode(RESET_N, OUTPUT);
    pinMode(RESET_5940, OUTPUT);
    pinMode(CELL485_DE, OUTPUT);


    digitalWrite(AD636_SEL,LOW );//REFERANCE
    digitalWrite(CS_5940, HIGH);
};

//HardwareSerial Serial1;
void AD5940_Main(void *parameters);
void AD5940_Main_init();

void wifiApmodeConfig()
{
}
void readnWriteEEProm()
{
  uint8_t ipaddr1;
  if (EEPROM.read(0) != 0x55)
  {
    systemDefaultValue.AlarmAmpere = 200;
    systemDefaultValue.alarmDiffCellVoltage = 200;
    systemDefaultValue.alarmHighCellVoltage = 14500;
    systemDefaultValue.alarmLowCellVoltage = 8500;
    systemDefaultValue.AlarmTemperature = 65;
    systemDefaultValue.cutoffHighCellVoltage = 14800;
    systemDefaultValue.cutoffLowCellVoltage = 6500;
    systemDefaultValue.GATEWAY =(uint32_t)IPAddress(192, 168, 0, 1); 
    systemDefaultValue.IPADDRESS =(uint32_t)IPAddress(192, 168, 0, 201); 
    systemDefaultValue.modbusId = 1;
    strncpy(systemDefaultValue.ssid ,"iptime_mbhong",20);
    strncpy(systemDefaultValue.ssid_password,"",10);
    systemDefaultValue.SUBNETMASK =(uint32_t)IPAddress(255, 255, 255, 0); 
    systemDefaultValue.TotalCellCount = 40;
    strncpy(systemDefaultValue.userid,"admin",10);
    strncpy(systemDefaultValue.userpassword,"admin",10);
    for(int i=0;i<40;i++){
      systemDefaultValue.voltageCompensation[i]=0;
      systemDefaultValue.impendanceCompensation[i]=0;
    }
    EEPROM.writeByte(0, 0x55);
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
  }
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
}
BluetoothSerial SerialBT;

void setRtcNewTime(RtcDateTime rtc){
  digitalWrite(CS_5940, HIGH);
  SPI.end();
  Rtc.Begin();
  delay(1);
  if (!Rtc.GetIsRunning())
  {
    printf("RTC was not actively running, starting now\r\n");
    Rtc.SetIsRunning(true);
  }
  else 
    printf("RTC was actively status running \r\n");
  // struct timeval tmv;
  // tmv.tv_usec = 0;
  // tmv.tv_sec = 0;
  //RtcDateTime now(tmv.tv_sec);
  Rtc.SetDateTime(rtc);
  myWire.end();
  SPI.begin(SCK,MISO,MOSI,CS_5940);
}
RtcDateTime getDs1302GetRtcTime(){
  digitalWrite(CS_5940, HIGH);
  SPI.end();
  Rtc.Begin();
  if (!Rtc.GetIsRunning())
  {
    printf("RTC was not actively running, starting now\r\n");
    Rtc.SetIsRunning(true);
  }
  else 
    printf("RTC was actively status running \r\n");
  delay(1);
  RtcDateTime nowTime = Rtc.GetDateTime();
  myWire.end();
  delay(1);
  SPI.begin(SCK,MISO,MOSI,CS_5940);
  return nowTime;
}
void setRtc()
{
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printf("\r\ncompiled time is %d/%d/%d %d:%d:%d\r\n",compiled.Year(),compiled.Month(),compiled.Day(),compiled.Hour(),compiled.Minute(),compiled.Second());
  if (!Rtc.IsDateTimeValid())
  {
    printf("RTC lost confidence in the DateTime!\r\n");
    //Rtc.SetDateTime(compiled);
  }
  else 
    printf("RTC available in the DateTime!\r\n");
  if (Rtc.GetIsWriteProtected())
  {
    printf("RTC was write protected, enabling writing now\r\n");
    Rtc.SetIsWriteProtected(false);
  }
  else 
    printf("RTC enabling writing \r\n");
  if (!Rtc.GetIsRunning())
  {
    printf("RTC was not actively running, starting now\r\n");
    Rtc.SetIsRunning(true);
  }
  else 
    printf("RTC was actively status running \r\n");
  if (!Rtc.GetIsRunning())
  {
    printf("RTC was not actively running, starting now\r\n");
    Rtc.SetIsRunning(true);
  }
  else 
    printf("RTC was actively status running \r\n");

  RtcDateTime now = Rtc.GetDateTime();
  printf("\r\nnow time is %d/%d/%d %d:%d:%d\r\n",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
  if (now < compiled)
  {
    printf("\r\nSet data with compiled time");
    //Rtc.SetDateTime(compiled);
  }
  struct timeval tmv;
  tmv.tv_sec = now.TotalSeconds();
  tmv.tv_usec = 0;
  //time_t toUnixTime = now.Unix32Time();
  //시스템의 시간도 같이 맞추어 준다.
  settimeofday(&tmv, NULL);
  gettimeofday(&tmv, NULL);
  now = RtcDateTime(tmv.tv_sec);
  //now = tmv.tv_sec;
  printf("\r\nset and reread time is %d/%d/%d %d:%d:%d\r\n",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());

  myWire.end();
}
#define SELECT_LCD do {digitalWrite(SERIAL_SEL_ADDR0,HIGH);digitalWrite(SERIAL_SEL_ADDR1,LOW); }while(0)
  
  
class ExtendSerial//: public HardwareSerial
{
  private:
    const uint8_t _addr0=23;
    const uint8_t _addr1=2;
    bool _addrValue0;
    bool _addrValue1;
  public:
    ExtendSerial(){// : HardwareSerial(uart_num) {
    //LCDSerial(uint8_t uart_num) : HardwareSerial(uart_num){
      pinMode(_addr0,OUTPUT);
      pinMode(_addr1,OUTPUT);
    };
    /* 송수신모드를 사용하기 위하여 사용산다. */
    /*  writeEnable true: 송신가능  */
    /* writeEnable false : 수신가능*/
    void selectCellModule(bool writeEnable){
      digitalWrite(_addr0,false);
      digitalWrite(_addr1,false); 
      digitalWrite(CELL485_DE, writeEnable);
    };
    void selectLcd(){
      digitalWrite(_addr0,true);
      digitalWrite(_addr1,false); 
    };
    void ReleaseLcd(){
      digitalWrite(_addr0,LOW);
      digitalWrite(_addr1,LOW); 
    };

};
#define NUM_VALUES 21
uint32_t request_time;
bool data_ready = false;
uint16_t values[2];
uint16_t cellModbusIdReceived;
void handleData(ModbusMessage response, uint32_t token) 
{
  // First value is on pos 3, after server ID, function code and length byte
  uint16_t offs = 3;
  // The device has values all as IEEE754 float32 in two consecutive registers
  // Read the requested in a loop
  for (uint8_t i = 0; i < 2; ++i) {
    offs = response.get(offs, values[i]);
    //Serial.printf("\n%Temperature %d",values[i]);
  }
  // Signal "data is complete"
  request_time = token;
  data_ready = true;
}
void handleError(Error error, uint32_t token) 
{
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  //LOG_E("Error response: %02X - %s\n", (int)me, (const char *)me);
  data_ready = false;
}

void setupModbusAgentForLcd(){
  //address는 항상 1이다.
  uint8_t address_485 = 1; 
  rtu485.begin(Serial2,9600,1);
  rtu485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  rtu485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  rtu485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);
  //rtu485.suspendTask();
  
  extrtu485.begin(Serial1,9600,1);
  extrtu485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  extrtu485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  extrtu485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);
  //extrtu485.suspendTask();

  // cellModbus.onDataHandler(&handleData);
  // cellModbus.onErrorHandler(&handleError);
  // cellModbus.setTimeout(2000);
  // cellModbus.begin(Serial2);
  // cellModbus.suspendTask();
};
ExtendSerial extendSerial;
int readRelayResponseData(uint8_t modbusId,uint8_t funcCode, uint8_t *buf,uint8_t len){
  uint16_t timeout;
  timeout = 300;
  uint16_t readCount=0;
  data_ready = false;
  while (timeout--)
  {
    if (Serial2.available())
    {
      buf[readCount++] = Serial2.read();
      //ESP_LOGI("REV","%02x ",buf[readCount -1]);
    };
    delay(1);
    if (readCount == len){
      //data를 받았다. 이제 id, command ,checksum 체크섬이 같은지 보자.
      //ESP_LOGI("REV","data received len is %02x ",len);
      if(buf[0] ==modbusId && buf[1] == funcCode &&  RTUutils::validCRC(buf,len)){
        data_ready = true;
        break;
      }
      else{
        //ESP_LOGI("REV"," modbusId %d funcCode %d  validCRC %x",buf[0] ,buf[1], RTUutils::validCRC(buf,len));
        data_ready = false;
        delay(3000);
        break;
      }
    }
  }
  if (data_ready)
  {
    ESP_LOGI("main","Data Good Recieved");
  }
  else
  {
    ESP_LOGI("main","----------------------------");
    ESP_LOGI("main","Receive Failed");
    ESP_LOGI("main","----------------------------");
  }
  while(Serial2.available())Serial2.read();

  return data_ready;
};
int makeRelayControllData(uint8_t *buf,uint8_t modbusId,uint8_t funcCode, uint16_t address, uint16_t len){
  uint16_t checkSum ;
  buf[0] =modbusId; buf[1] = funcCode;
  buf[2] = (uint8_t)((address & 0xff00) >>8 ); 
  buf[3] = (uint8_t)(address & 0x00ff); // Address는 0부터 
  buf[4] = (uint8_t)((len & 0xff00) >>8) ; 
  buf[5] = (uint8_t)(len & 0x00ff);  //  갯수는 2개
  checkSum =  RTUutils::calcCRC(buf,6);
  buf[6] = checkSum & 0x00FF;
  buf[7] = checkSum >> 8    ;

  extendSerial.selectCellModule(false);
  while(Serial2.available())Serial2.read();
  extendSerial.selectCellModule(true);
  Serial2.write(buf,8);
  Serial2.flush();
  extendSerial.selectCellModule(false);
  return 1;
}

bool sendSelectBattery(uint8_t modbusId)
{
  //Coil 명령를 사용하며 
  //1. 0xFF 명령으로 전체 OUT명령을 준다. 
  //2. modbusID를 켠다. 
  //3. 제대로 켜졌는지 다시 읽어본다. 
  //4. modbusID+1를 켠다 
  //5. 제대로 켜졌는지 다시 읽어본다. 

  selecectedCellNumber = modbusId;
  uint16_t checkSum ;
  ESP_LOGI("main","request");
  rtu485.suspendTask();
  uint8_t buf[256];

  ESP_LOGI("main","전체 릴레이를 끄자(%d)",buf[3]);
  //makeRelayControllData(buf,0xFF,05,0,0xFF00); // 0xff BROADCAST
  makeRelayControllData(buf,0xFF,05,0,0x00); // 0xff BROADCAST
  delay(100);
  //makeRelayControllData(buf,0xFF,05,1,0xFF00); // 0xff BROADCAST
  makeRelayControllData(buf,0xFF,05,1,0x00); // 0xff BROADCAST
  delay(500);
  makeRelayControllData(buf,modbusId,01,0,2); // Read coil data 2 개 

  extendSerial.selectCellModule(false);  //읽기 모드로 전환
  uint16_t readCount = readRelayResponseData(modbusId,1, buf,6); //buf[3]이 Relay 데이타 이다.

  makeRelayControllData(buf,modbusId,05,0,0xFF00); // 0xff BROADCAST
  delay(100);
  makeRelayControllData(buf,modbusId+1,05,1,0xFF00); // 0xff BROADCAST
  delay(200);

  extendSerial.selectLcd();
  rtu485.resumeTask();
  return data_ready;

}
int makeTemperatureData(uint8_t *buf,uint8_t modbusId,uint8_t funcCode, uint16_t address, uint16_t len){
  uint16_t checkSum ;
  buf[0] =modbusId; 
  buf[1] = funcCode;
  buf[2] = (uint8_t)((address & 0xff00) >>8 ); 
  buf[3] = (uint8_t)(address & 0x00ff); // Address는 0부터 
  buf[4] = (uint8_t)((len & 0xff00) >>8) ; 
  buf[5] = (uint8_t)(len & 0x00ff);  //  갯수는 2개
  checkSum =  RTUutils::calcCRC(buf,6);
  buf[6] = checkSum & 0x00FF;
  buf[7] = checkSum >> 8    ;

  extendSerial.selectCellModule(false);
  while(Serial2.available())Serial2.read();
  extendSerial.selectCellModule(true);
  Serial2.write(buf,8);
  Serial2.flush();
  extendSerial.selectCellModule(false);
  return 1;
}
bool sendGetMoubusTemperature(uint8_t modbusId, uint8_t fCode)
{
  uint16_t checkSum ;
  ESP_LOGI("main","request");
  rtu485.suspendTask();
  uint8_t buf[256];
  data_ready = false;
  makeTemperatureData(buf,modbusId,fCode,0,2);
  // buf[0] =modbusId;
  // buf[1] = fCode;
  // buf[2] = 0;
  // buf[3] = 0;
  // buf[4] = 0;
  // buf[5] = 2;
  // checkSum =  RTUutils::calcCRC(buf,6);
  // buf[6] = checkSum & 0x00FF;
  // buf[7] = checkSum >> 8    ;

  extendSerial.selectCellModule(false);
  uint16_t readCount = readRelayResponseData(modbusId,1, buf,9); //buf[3]이 Relay 데이타 이다.
  // while(Serial2.available())Serial2.read();
  // extendSerial.selectCellModule(true);
  // Serial2.write(buf,8);
  // Serial2.flush();
  // extendSerial.selectCellModule(false);
  // uint16_t timeout;
  // timeout = 100;
  // while (timeout--)
  // {
  //   if (Serial2.available())
  //   {
  //     buf[readCount++] = Serial2.read();
  //   };
  //   delay(1);
  //   if (readCount == 9){
  //     if(buf[0] ==modbusId && buf[1] == fCode &&  RTUutils::validCRC(buf,6)){
  //     data_ready = true;
  //     break;
  //     }
  //   }
  // }
  if (data_ready)
  {
    uint16_t value = buf[3]*256  + buf[4] ;
    ESP_LOGI("modbus","%d:%02x %02x Temperature %d",modbusId-1,buf[3],buf[4], value);
    cellvalue[modbusId - 1].temperature = value / 100;
  }
  else
  {
    ESP_LOGI("modbus","----------------------------");
    ESP_LOGI("modbus","Receive Failed");
    ESP_LOGI("modbus","----------------------------");
  }
  // while(Serial2.available())Serial2.read();
  extendSerial.selectLcd();
  rtu485.resumeTask();
  return data_ready;

};
void setup(){
  EEPROM.begin(1000);
  readnWriteEEProm();
  pinsetup();
  Serial.begin(BAUDRATE);
  // 내부의 LCD와 셀의 온도및 릴레이를 위해 사용한다.
  Serial1.begin(BAUDRATE,SERIAL_8N1,SERIAL_RX1 ,SERIAL_TX1 );
  //외부 485통신에 사용한다.
  Serial2.begin(BAUDRATE,SERIAL_8N1,SERIAL_RX2 ,SERIAL_TX2 );
  //ExtendSerial.selectCellModule(true);
  extendSerial.selectLcd();  //232통신이다 
                             // 485가 enable가 된다고 해도 
                             // 그쪽으로는 출력이 되지 않으므로 상관이 없다.

  for(int i=0;i<40;i++){
    cellvalue[i].voltage = 0;
    cellvalue[i].impendance=0; 
    cellvalue[i].temperature =0;
    cellvalue[i].voltageCompensation= 0;
    cellvalue[i].impendanceCompensation= 0;
  }

  setupModbusAgentForLcd();
  SerialBT.begin("TIMP_Device_1");
  wifiApmodeConfig();
  lsFile.littleFsInitFast(0);
  setRtc();
  SPI.setFrequency(spiClk );
  SPI.begin(SCK,MISO,MOSI,CS_5940);
  pinMode(SS, OUTPUT); //VSPI SS -> 아니다..이것은 리셋용이다.

  AD5940_MCUResourceInit(0);
  AD5940_Main_init();
  delay(1000);
  ESP_LOGI(TAG, "System Started");

  //xTaskCreate(NetworkTask,"NetworkTask",5000,NULL,1,h_pxNetworkTask); //PCB 패턴문제로 사용하지 않는다.
  xTaskCreate(blueToothTask,"blueToothTask",5000,NULL,1,h_pxblueToothTask);
};
static unsigned long previousSecondmills = 0;
static int everySecondInterval = 1000;

static int Interval_3Second = 3000;
static unsigned long previous_3Secondmills = 0;

static int Interval_5Second = 5000;
static unsigned long previous_5Secondmills = 0;

static int Interval_30Second = 30000;
static unsigned long previous_30Secondmills = 0;

static int Interval_60Second = 60000;
static unsigned long previous_60Secondmills = 0;

static unsigned long now;
//각각의 시간은 병렬로 수행된다.

uint8_t modbusId=1;
BatDeviceInterface batDevice;
void loop(void)
{
  void *parameters;
  now = millis(); 
  if ((now - previousSecondmills > everySecondInterval))
  {
    ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
    previousSecondmills = now;
  }
  bool bRet;
  if ((now - previous_3Secondmills > Interval_3Second))
  {
    previous_3Secondmills= now;
  }
  if ((now - previous_5Secondmills > Interval_5Second))
  {
    for(int i=1;i<INSTALLED_CELLS;i++){
      sendGetMoubusTemperature(i,04);
      sendSelectBattery(i);
      // AD5940_Main(parameters);  //for test 무한 루프
      // time_t startRead = millis();
      float batVoltage =  batDevice.readBatAdcValue(600);
      cellvalue[i - 1].voltage= batVoltage ;  //구조체에 값을 적어 넣는다
      // time_t endRead = millis();// take 300ms
      // ESP_LOGI("Voltage","Bat Voltage is : %3.3f (%ldmilisecond)",batVoltage,endRead-startRead);
    }
    //modbusId = modbusId > 4 ? 1:modbusId;
    previous_5Secondmills= now;
  }
  if ((now - previous_30Secondmills > Interval_30Second))
  {

    previous_30Secondmills= now;
  }
  if ((now - previous_60Secondmills > Interval_60Second))
  {
    for(int i=1;i< INSTALLED_CELLS;i++)
    {
      sendGetMoubusTemperature(i,04);
    }
    // sendSelectBattery(1);
    // AD5940_Main(parameters);  //for test 무한 루프
    previous_60Secondmills= now;
  }
  delay(10);
}