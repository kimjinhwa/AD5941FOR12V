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

// #include <esp_int_wdt.h>
// #include <esp_task.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT 60 
// 기본 vSPI와 일치한다
#define VSPI_MISO   MISO  // IO19
#define VSPI_MOSI   MOSI  // IO 23
#define VSPI_SCLK   SCK   // IO 18
#define VSPI_SS     15    // IO 15

// //SPIClass * vspi = NULL;

//SPIClass SPI;
static const int spiClk = 1000000; // 1 MHz
static char TAG[] ="Main";
//TaskHandle_t *h_pxNetworkTask;
TaskHandle_t *h_pxblueToothTask;
nvsSystemSet systemDefaultValue;
ThreeWire myWire(13, 14, 33); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
ModbusServerRTU LcdCell485(2000,CELL485_DE);// LCD를 위하여 사용한다.
ModbusServerRTU external485(2000,EXT_485EN_1);
//ModbusClientRTU cellModbus(CELL485_DE);
uint8_t selecectedCellNumber =0;

_cell_value cellvalue[MAX_INSTALLED_CELLS];


void AD5940_ShutDown();
void pinsetup()
{
    pinMode(READ_BATVOL, INPUT);

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
    systemDefaultValue.runMode = 0;  // 0: manual 0x01 : onlyVoltate Audo, 0x03 : Voltage & Impedance 
    systemDefaultValue.AlarmAmpere = 2000;  // 200A
    systemDefaultValue.alarmDiffCellVoltage = 200;  //200mV
    systemDefaultValue.alarmHighCellVoltage = 1450;  //14.5V
    systemDefaultValue.alarmLowCellVoltage = 850; //8.5V
    systemDefaultValue.AlarmTemperature = 65+40;
    systemDefaultValue.cutoffHighCellVoltage = 14800;
    systemDefaultValue.cutoffLowCellVoltage = 6500;
    systemDefaultValue.GATEWAY =(uint32_t)IPAddress(192, 168, 0, 1); 
    systemDefaultValue.IPADDRESS =(uint32_t)IPAddress(192, 168, 0, 201); 
    systemDefaultValue.modbusId = 1;
    strncpy(systemDefaultValue.ssid ,"iptime_mbhong",20);
    strncpy(systemDefaultValue.ssid_password,"",10);
    systemDefaultValue.SUBNETMASK =(uint32_t)IPAddress(255, 255, 255, 0); 
    systemDefaultValue.installed_cells= 40;
    strncpy(systemDefaultValue.userid,"admin",10);
    strncpy(systemDefaultValue.userpassword,"admin",10);
    for(int i=0;i<40;i++){
      systemDefaultValue.voltageCompensation[i]=0;
      systemDefaultValue.impendanceCompensation[i]=0;
    }
    systemDefaultValue.real_Cal = -33410.0f;
    systemDefaultValue.image_Cal = 35511.0f;
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
  vTaskDelay(1);
  if (!Rtc.GetIsRunning())
  {
    printf("RTC was not actively running, starting now\r\n");
    Rtc.SetIsRunning(true);
  }
  else 
    printf("RTC was actively status running \r\n");
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
  vTaskDelay(1);
  RtcDateTime nowTime = Rtc.GetDateTime();
  myWire.end();
  vTaskDelay(1);
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
    printf("\r\nSet data with compiled time"); //Rtc.SetDateTime(compiled);
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
//bool data_ready = false;
uint16_t values[2];
uint16_t cellModbusIdReceived;

void setupModbusAgentForLcd(){
  //address는 항상 1이다.
  uint8_t address_485 = 1; 
  LcdCell485.useStopControll =1;
  LcdCell485.begin(Serial2,BAUDRATESERIAL1,1);
  LcdCell485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  LcdCell485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  LcdCell485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);

  external485.useStopControll =0;
  external485.begin(Serial1,BAUDRATE,1);
  external485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  external485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  external485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);

};
ExtendSerial extendSerial;
int readResponseData(uint8_t modbusId,uint8_t funcCode, uint8_t *buf,uint8_t len,uint16_t timeout){
  //uint16_t timeout; //timeout = 300;
  uint16_t readCount=0;
  bool data_ready = false;
  data_ready = false;
  while (timeout--)
  {
    if (Serial2.available())
    {
      buf[readCount++] = Serial2.read();
      //ESP_LOGI("REV","%02x ",buf[readCount -1]);
    };
    delayMicroseconds(100);
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
        //vTaskDelay(3000);
        break;
      }
    }
  }
  if (data_ready)
  {
    //ESP_LOGI("main","Data Good Recieved");
  }
  else
  {
    for (int i = 0; i < readCount; i++)
    {
      Serial.printf("%d:%02x ",i ,buf[readCount]);
    }
    ESP_LOGI("main", "Receive Failed %d received",readCount);
  }
  //리턴하기 전에 Garbage가 있으면 정리한다.
  while(Serial2.available())
  {
    int c = Serial2.read();
    ESP_LOGI(TAG,"Garbage Date : %02x",c);
    vTaskDelay(1);
  }
  return data_ready;
};
int makeRelayControllData(uint8_t *buf,uint8_t modbusId,uint8_t funcCode, uint16_t address, uint16_t len)
{
  uint16_t checkSum ;
  buf[0] =modbusId; buf[1] = funcCode;
  buf[2] = (uint8_t)((address & 0xff00) >>8 ); 
  buf[3] = (uint8_t)(address & 0x00ff); // Address는 0부터 
  buf[4] = (uint8_t)((len & 0xff00) >>8) ; 
  buf[5] = (uint8_t)(len & 0x00ff);  //  갯수는 2개
  checkSum =  RTUutils::calcCRC(buf,6);
  buf[6] = checkSum & 0x00FF;
  buf[7] = checkSum >> 8    ;

  extendSerial.selectCellModule(false);//change to read Mode
  while(Serial2.available()){Serial2.read();vTaskDelay(1);};// clear comm buffer
  extendSerial.selectCellModule(true); vTaskDelay(1);
  Serial2.write(buf,8);
  Serial2.flush();
  extendSerial.selectCellModule(false);  //change to read Mode
  return 1;
}
/* All Off will return 0  
*   else return value;
*/
uint16_t checkAlloff(uint32_t *failedBatteryNumberH,uint32_t *failedBatteryNumberL)
{
  uint16_t  totalRelayCount;
  uint16_t isOK ;
  uint32_t temp32H,temp32L;
  *failedBatteryNumberH= 0;
  *failedBatteryNumberL= 0;
  uint8_t buf[64];
  for (int modbusId = 1; modbusId <= systemDefaultValue.installed_cells; modbusId++)
  {
    vTaskDelay(10);
    makeRelayControllData(buf, modbusId, READ_COIL, 0, 2);     // Read coil data 2 개
    extendSerial.selectCellModule(false);                      // 읽기 모드로 전환
    isOK = readResponseData(modbusId, READ_COIL, buf, 6, 3000); // buf[3]이 Relay 데이타 이다.
    if (isOK == 1)
    {
      if (modbusId < 32)
      {
        temp32L = 1;
        temp32L = temp32L << (modbusId - 1);
        *failedBatteryNumberL |= temp32L;
      }
      else
      {
        temp32H = 1;
        temp32H = temp32H << (modbusId - 1);
        *failedBatteryNumberH |= temp32H;
      }
    }
    else
    {
      if (modbusId < 32)
      {
        temp32L  = 0;
        temp32L = ~(1 << (modbusId - 1));
        *failedBatteryNumberL &= temp32L;
      }
      else
      {
        temp32H = 0;
        temp32H = ~(1 << (modbusId - 1));
        *failedBatteryNumberH &= temp32H;
      }
      ESP_LOGI("OFF RELAY","%d battery not response..", modbusId);
    }
    totalRelayCount += buf[3];
  }
  return totalRelayCount;
}
/* 셀을 선택한다. modbusId는 1부터 시작하며 설치되어 있는 배터리의 수보다 작아야 한다. 
* 
*/
bool sendSelectBattery(uint8_t modbusId)
{
  // Coil 명령를 사용하며
  // 1. 0xFF 명령으로 전체 OUT명령을 준다.
  // 2. 현재 설치되어 있는 모든셀들이 통신 가능하여야 하고 릴레이 설정값을 0 을 갖고 있어야 한다.  
  // 3. modbusID를 켠다
  // 4. modbusID+1를 켠다
  // 에러가 없다면 정상적으로 켜졌을 것이고, 확인 루틴은 다음번에 셀을 선택할 때 한다 
  AD5940_ShutDown();  // 전류의 흐름을 없애기 위하여 혹시 파형을 출력 중이면 정지 시킨다.
  selecectedCellNumber = modbusId;
  uint16_t checkSum;
  LcdCell485.suspendTask();
  uint8_t buf[64];

  makeRelayControllData(buf, 0xFF, WRITE_COIL, 0, 0x00); // 0xff BROADCAST
  vTaskDelay(100);
  
  makeRelayControllData(buf, 0xFF, WRITE_COIL, 1, 0x00); // 0xff BROADCAST
  vTaskDelay(500);
  makeRelayControllData(buf, modbusId, READ_COIL, 0, 2); // Read coil data 2 개

  extendSerial.selectCellModule(false);                                     // 읽기 모드로 전환
  uint16_t readCount = readResponseData(modbusId, READ_COIL, buf, 6, 3000); // buf[3]이 Relay 데이타 이다.

  uint32_t failedBatteryH, failedBatteryL;
  uint16_t retValue;
  retValue = checkAlloff(&failedBatteryH, &failedBatteryL);
  extendSerial.selectCellModule(false);                                     // 읽기 모드로 전환
  vTaskDelay(100);  //이값을 주고 나서야 릴레이가 제대로 동작하였다.
  // retValue는 반드시 0이어야 하고...
  // 총셀은 20셀으므로 상위 바이트는 0X000F
  //  하위 바이트는 0XFFFF이어야 한다.
  uint32_t checkH = 0;
  uint32_t checkL = 0;
  for (int i = 0; i < systemDefaultValue.installed_cells; i++)
  {
    if (i < 32)
      checkL |= (1U << i);
    else
      checkH |= (1U << i);
  }
  ESP_LOGI(TAG, "Relay ON State :%d %04x %04x %04x %04x", retValue, checkH, checkL, failedBatteryH, failedBatteryL);
  if (retValue == 0 && checkH == failedBatteryH && checkL == failedBatteryL)
  {
    makeRelayControllData(buf, modbusId, WRITE_COIL, 0, 0xFF00); // 해당 셀을 ON 시킨다
    extendSerial.selectCellModule(false);                                     // 읽기 모드로 전환
    readCount = readResponseData(modbusId, WRITE_COIL, buf, 8, 3000); // buf[3]이 Relay 데이타 이다.
    if(readCount == 1){
      ESP_LOGI(TAG,"relay %d Minus(-) ON ",modbusId);
    }
    vTaskDelay(100);
    makeRelayControllData(buf, modbusId + 1, WRITE_COIL, 1, 0xFF00); // 해당 셀을 ON 시킨다
    readCount = readResponseData(modbusId+1, WRITE_COIL, buf, 8, 3000); // buf[3]이 Relay 데이타 이다.
    if(readCount == 1){
      ESP_LOGI(TAG,"relay %d Plus(+) ON ",modbusId+1);
    }
    vTaskDelay(200);
  }
  else
  {
    ESP_LOGI(TAG,"All relay off fail or communication fails ");
  }
  extendSerial.selectLcd();
  LcdCell485.resumeTask();
  return readCount ;
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
  while(Serial2.available()){
    int c = Serial2.read();
    ESP_LOGI(TAG,"Garbage Date : %02x",c);
    vTaskDelay(1);
  }
  extendSerial.selectCellModule(true);
  vTaskDelay(5);
  Serial2.write(buf,8);
  Serial2.flush();
  extendSerial.selectCellModule(false);
  return 1;
}
uint16_t sendGetMoubusTemperature(uint8_t modbusId, uint8_t fCode)
{
  uint16_t checkSum ;
  uint16_t value ;
  bool data_ready = false;
  //ESP_LOGI("main","request");
  LcdCell485.suspendTask();
  vTaskDelay(100);
  uint8_t buf[64];
  data_ready = false;
  makeTemperatureData(buf,modbusId,fCode,0,2);
  extendSerial.selectCellModule(false);
  data_ready  = readResponseData(modbusId,fCode, buf,9,3000); 
  if (data_ready)
  {
    value = buf[3]*256  + buf[4] ;
    ESP_LOGI("modbus","%d:%02x %02x Temperature %d",modbusId-1,buf[3],buf[4], value);
    cellvalue[modbusId - 1].temperature = value ;
  }
  else
  {
    value =0;
    ESP_LOGI("modbus","Receive Failed");
  }
  // while(Serial2.available())Serial2.read();
  extendSerial.selectLcd();
  LcdCell485.resumeTask();
  return value;

};
void setup()
{
  EEPROM.begin(1000);
  readnWriteEEProm();
  pinsetup();
  Serial.begin(BAUDRATE);
  // 외부 485통신에 사용한다.
  Serial1.begin(BAUDRATE, SERIAL_8N1, SERIAL_RX1, SERIAL_TX1);
  // 내부의 LCD와 셀의 온도및 릴레이를 위해 사용한다.
  Serial2.begin(BAUDRATESERIAL1, SERIAL_8N1, SERIAL_RX2, SERIAL_TX2);
  // ExtendSerial.selectCellModule(true);
  extendSerial.selectLcd(); // 232통신이다
                            //  485가 enable가 된다고 해도
                            //  그쪽으로는 출력이 되지 않으므로 상관이 없다.

  for (int i = 0; i < 40; i++)
  {
    cellvalue[i].voltage = 0;
    cellvalue[i].impendance = 0;
    cellvalue[i].temperature = 0;
    cellvalue[i].voltageCompensation = 0;
    cellvalue[i].impendanceCompensation = 0;
  }

  setupModbusAgentForLcd();
  String bleName ="TIMP_Dev_"; 
  bleName += systemDefaultValue.modbusId;
  SerialBT.begin(bleName.c_str() );
  wifiApmodeConfig();
  lsFile.littleFsInitFast(0);
  setRtc();
  SPI.setFrequency(spiClk);
  SPI.begin(SCK, MISO, MOSI, CS_5940);
  pinMode(SS, OUTPUT); // VSPI SS -> 아니다..이것은 리셋용이다.

  AD5940_MCUResourceInit(0);
  AD5940_Main_init();
  vTaskDelay(1000);
  ESP_LOGI(TAG, "System Started at %s mode", systemDefaultValue.runMode == 0 ? "Manual" : "Auto");
  ESP_LOGI(TAG,"\nEEPROM installed Bat number %d", systemDefaultValue.installed_cells);
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  // esp_task_wdt_reset();
  // esp_task_wdt_init(5, true); // WDT를 활성화하고, panic 핸들러를 사용하여 리셋합니다.
  // esp_task_wdt_add(NULL); // 현재 태스크를 WDT에 추가합니다.
  // xTaskCreate(NetworkTask,"NetworkTask",5000,NULL,1,h_pxNetworkTask); //PCB 패턴문제로 사용하지 않는다.
  xTaskCreate(blueToothTask, "blueToothTask", 5000, NULL, 1, h_pxblueToothTask);
  ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
  // for (int i = 1; i < systemDefaultValue.installed_cells; i++)
  // {
  //   sendGetMoubusTemperature(i, 04);
  //   ESP_LOGI(TAG,"Selecet Module %d",i);
  // }
  AD5940_ShutDown();
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

//uint8_t globalModbusId =1;
BatDeviceInterface batDevice;
uint8_t impedanceCellPosition=1;
void loop(void)
{
  bool bRet;
  void *parameters;
  now = millis(); 
  esp_task_wdt_reset();
  if ((now - previousSecondmills > everySecondInterval))
  {
    previousSecondmills = now;
  }
  if ((now - previous_3Secondmills > Interval_3Second))
  {
    previous_3Secondmills= now;
  }
  if ((now - previous_5Secondmills > Interval_5Second))
  {
    if (systemDefaultValue.runMode != 0)  // 자동 모드에서만 실행한다
    for (int i = 1; i <= systemDefaultValue.installed_cells ; i++)
    {
      sendGetMoubusTemperature(i, READ_INPUT_REGISTER);
      esp_task_wdt_reset();
      sendSelectBattery(i);
      time_t startRead = millis();
      float batVoltage = 0.0;
      batVoltage = batDevice.readBatAdcValue(600);
      cellvalue[i - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
      time_t endRead = millis();             // take 300ms
      ESP_LOGI("Voltage", "Bat Voltage is : %3.3f (%ldmilisecond)", batVoltage, endRead - startRead);
      if (batVoltage > 2.0)
      {
        if (systemDefaultValue.runMode == 3)
          AD5940_Main(parameters); // for test 무한 루프
      }
      vTaskDelay(1000);
    }
 //   globalModbusId = globalModbusId > 4 ? 1 : globalModbusId ;
    previous_5Secondmills = now;
  }
  if ((now - previous_30Secondmills > Interval_30Second))
  {
    previous_30Secondmills= now;
  }
  if ((now - previous_60Secondmills > Interval_60Second))
  {
    // sendGetMoubusTemperature(impedanceCellPosition,04);
    // sendSelectBattery(impedanceCellPosition);//selecectedCellNumber를 변화 시킨다
    // AD5940_Main(parameters);  
    // impedanceCellPosition++;
    // if(impedanceCellPosition >= INSTALLED_CELLS)impedanceCellPosition =1;
    previous_60Secondmills= now;
  }
  vTaskDelay(100);
}

    // pinMode(SCK, OUTPUT);
    // pinMode(MISO, OUTPUT);
    // pinMode(MOSI, OUTPUT);
  // struct timeval tmv;
  // tmv.tv_usec = 0;
  // tmv.tv_sec = 0;
  //RtcDateTime now(tmv.tv_sec);
// void handleData(ModbusMessage response, uint32_t token) 
// {
//   // First value is on pos 3, after server ID, function code and length byte
//   uint16_t offs = 3;
//   // The device has values all as IEEE754 float32 in two consecutive registers
//   // Read the requested in a loop
//   for (uint8_t i = 0; i < 2; ++i) {
//     offs = response.get(offs, values[i]);
//     //Serial.printf("\n%Temperature %d",values[i]);
//   }
//   // Signal "data is complete"
//   request_time = token;
//   data_ready = true;
// }
// void handleError(Error error, uint32_t token) 
// {
//   // ModbusError wraps the error code and provides a readable error message for it
//   ModbusError me(error);
//   //LOG_E("Error response: %02X - %s\n", (int)me, (const char *)me);
//   data_ready = false;
// }
  // cellModbus.onDataHandler(&handleData);
  // cellModbus.onErrorHandler(&handleError);
  // cellModbus.setTimeout(2000);
  // cellModbus.begin(Serial2);