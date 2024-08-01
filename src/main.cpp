#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <wifi.h>
#include <driver/adc.h>
#include "filesystem.h"
#include "mainGrobal.h"
#include "SimpleCLI.h"

#ifdef WEBOTA
#include <WebServer.h>
#include "esp32WebOTA.h"
#endif

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
#include "modbusCellModule.h"

// #include <esp_int_wdt.h>
// #include <esp_task.h>
#include <esp_task_wdt.h>

#define RELAY_OFF   1  
#define RELAY_ON    0 

#define WDT_TIMEOUT 60 
// 기본 vSPI와 일치한다
#define VSPI_MISO   MISO  // IO19
#define VSPI_MOSI   MOSI  // IO 23
#define VSPI_SCLK   SCK   // IO 18
#define VSPI_SS     15    // IO 15

#define NUM_VALUES 21

#define SELECT_LCD do {digitalWrite(SERIAL_SEL_ADDR0,HIGH);digitalWrite(SERIAL_SEL_ADDR1,LOW); }while(0)
// //SPIClass * vspi = NULL;

//SPIClass SPI;
static const int spiClk = 1000000; // 1 MHz
static char TAG[] ="Main";

TaskHandle_t *h_pxblueToothTask;
nvsSystemSet systemDefaultValue;
ThreeWire myWire(13, 14, 33); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
ModbusServerRTU LcdCell485(2000,CELL485_DE);// LCD를 위하여 사용한다.
ModbusServerRTU external485(2000,EXT_485EN_1);

uint32_t request_time;
uint16_t values[2];
uint16_t cellModbusIdReceived;
ExtendSerial extendSerial;
//ModbusClientRTU cellModbus(CELL485_DE);
uint8_t selecectedCellNumber =0;

_cell_value cellvalue[MAX_INSTALLED_CELLS];

extern SimpleCLI simpleCli;

BluetoothSerial SerialBT;


#ifdef WEBOTA
extern WebServer webServer;
#endif

void AD5940_ShutDown();
void pinsetup()
{
    pinMode(READ_BATVOL, INPUT);

    pinMode(AD5940_ISR, OUTPUT);
    pinMode(SERIAL_SEL_ADDR0, OUTPUT);
    pinMode(SERIAL_SEL_ADDR1, OUTPUT);
    pinMode(SERIAL_TX2 , OUTPUT);
    pinMode(LED_OP, OUTPUT);
    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);

    pinMode(RTC1305_EN, OUTPUT);
    pinMode(AD636_SEL, OUTPUT);
    pinMode(CS_5940, OUTPUT);
    pinMode(EXT_485EN_1, OUTPUT);
    pinMode(RESET_N, OUTPUT);
    pinMode(RESET_5940, OUTPUT);
    pinMode(CELL485_DE, OUTPUT);


    digitalWrite(AD636_SEL,LOW );//REFERANCE
    digitalWrite(CS_5940, HIGH);
    digitalWrite(RELAY_1, RELAY_OFF   );
    digitalWrite(RELAY_2, RELAY_OFF   );
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
    systemDefaultValue.AlarmTemperature = 65;
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
  
  

void setupModbusAgentForLcd(){
  //address는 항상 1이다.
  uint8_t address_485 = systemDefaultValue.modbusId; 
  LcdCell485.useStopControll =1;
  LcdCell485.begin(Serial2,BAUDRATESERIAL2,1);
  LcdCell485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  LcdCell485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  LcdCell485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);

  external485.useStopControll =0;
  external485.begin(Serial1,BAUDRATE,1);
  external485.registerWorker(address_485,READ_COIL,&FC01);
  external485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  external485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  external485.registerWorker(address_485,WRITE_COIL,&FC05);
  external485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);

};

/* All Off will return 0  
*   else return value;
* Ret : 모든 릴레이의 합의 값이다.
*/
/* 셀을 선택한다. modbusId는 1부터 시작하며 설치되어 있는 배터리의 수보다 작아야 한다. 
* 
*/

int measuredImpedance_1[20]={
    267,265,292,255,271,
    274,383,307,277,272,
    262,267,294,278,270,
    285,259,289,262,254
  };
int measuredImpedance_2[20]={
    334,337,349,340,345,
    334,331,350,337,339,
    334,332,349,337,328,
    343,341,358,330,334
  };
int measuredVoltage_1[20]={
    1351,1321,1317,1311,1314,
    1320,1322,1320,1321,1320,
    1325,1339,1338,1338,1344,
    1353,1343,1359,1343,1352
  };
int measuredVoltage_2[20]={
    1339,1340,1340,1339,1338,
    1335,1335,1336,1336,1335,
    1334,1334,1333,1334,1334,
    1334,1334,1332,1332,1333
  };
void initCellValue()
{
  if (systemDefaultValue.modbusId == 1)
  {
    for(int i=0;i<20;i++){
      cellvalue[i].impendance = float(measuredImpedance_1[i])/100.0f;
    }
  }
  else
  {
    for(int i=0;i<20;i++){
      cellvalue[i].impendance = float(measuredImpedance_2[i])/100.0f;
    }
  }
}
void setup()
{

  EEPROM.begin(sizeof(nvsSystemSet)+1);
  readnWriteEEProm();
  pinsetup();
  Serial.begin(BAUDRATE);
  // 외부 485통신에 사용한다.
  Serial1.begin(BAUDRATE, SERIAL_8N1, SERIAL_RX1, SERIAL_TX1);
  esp_reset_reason_t resetReson =  esp_reset_reason();
  String strResetReason="System booting reason is  ";
  bool dataReload=false;
  switch (resetReson )
  {
  case ESP_RST_UNKNOWN:
    strResetReason +=" Reset reason can not be determined";
    dataReload = true;
    break;
  case ESP_RST_POWERON:
    strResetReason +=" Reset due to power-on event";
    dataReload = true;
    break;
  case ESP_RST_EXT:
    strResetReason +=" Reset by external pin (not applicable for ESP32)";
    break;
  case ESP_RST_SW:
    strResetReason +=" Software reset via esp_restart";
    dataReload = true;
    break;
  case ESP_RST_PANIC:
    strResetReason +=" Software reset due to exception/panic";
    dataReload = true;
    break;
  case ESP_RST_INT_WDT:
    strResetReason +=" Reset (software or hardware) due to interrupt watchdog";
    dataReload = true;
    break;
  case ESP_RST_TASK_WDT:
    strResetReason +=" Reset due to task watchdog";
    dataReload = true;
    break;
  case ESP_RST_WDT:
    strResetReason +=" Reset due to other watchdogs";
    dataReload = true;
    break;
  case ESP_RST_DEEPSLEEP:
    strResetReason +=" Reset after exiting deep sleep mode";
    dataReload = true;
    break;
  case ESP_RST_BROWNOUT:
    strResetReason +=" Brownout reset (software or hardware)";
    dataReload = true;
    break;
  case ESP_RST_SDIO:
    strResetReason +=" Reset over SDIO";
    dataReload = true;
    break;
  default:
    break;
  }
  Serial.println("--------------------------------");
  Serial.println(strResetReason);
  Serial.println("--------------------------------");

  // 내부의 LCD와 셀의 온도및 릴레이를 위해 사용한다.
  Serial2.begin(BAUDRATESERIAL2, SERIAL_8N1, SERIAL_RX2, SERIAL_TX2);
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
  modbusCellModuleSetup();
  String bleName ="TIMP_"; 
  String WifiAddress = WiFi.macAddress();
  bleName += WifiAddress ;
  bleName += "_";
  bleName += systemDefaultValue.modbusId;
  SerialBT.begin(bleName.c_str() );
  wifiApmodeConfig();
  lsFile.littleFsInitFast(0);
  setRtc();
  lsFile.writeLogString(strResetReason);
  if(dataReload)lsFile.readCellDataLog(1);

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
  // xTaskCreate(NetworkTask,"NetworkTask",5000,NULL,1,h_pxNetworkTask); //PCB 패턴문제로 사용하지 않는다.
  xTaskCreate(blueToothTask, "blueToothTask", 4000, NULL, 1, h_pxblueToothTask);
  ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
  AD5940_ShutDown();

#ifdef WEBOTA
  webInit(); 
#endif
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
static timeval tmv;
int16_t logForHour=0;
uint32_t loopCount=0;
void loop(void)
{
  bool bRet;
  void *parameters;
#ifdef WEBOTA
  if(WiFi.isConnected()) webServer.handleClient();
#endif
  parameters = simpleCli.outputStream;
  now = millis(); 
  esp_task_wdt_reset();
  //moubusMouduleProc();
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
    {
      for (int i = 1; i <= systemDefaultValue.installed_cells; i++)
      {
        parameters = simpleCli.outputStream;
        //modbusRequestModule.addToQueue(millis(), i, READ_INPUT_REGISTER, 0, 3);
        //TODO: 임시로 막는 다>
        sendGetModbusModuleData(millis(), i, READ_INPUT_REGISTER, 0, 3);
        esp_task_wdt_reset();
        //TODO: 아래의 펑션은 MODBUS 루틴을 변경하기 위해 임시로 막는다
        SelectBatteryMinusPlus(i);
        // time_t startRead = millis();
        // float batVoltage = 0.0;
        // batVoltage = batDevice.readBatAdcValue(i, 600);
        // if (batVoltage > 18.0)
        //   batVoltage = 0.0;
        // cellvalue[i - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
        // time_t endRead = millis();             // take 300ms
        // ESP_LOGI("Voltage", "Bat(%d) Voltage is : %3.3f (%ldmilisecond)", i,batVoltage, endRead - startRead);
        // simpleCli.outputStream->printf("\nBat(%i) Voltage is : %3.3f (%ldmilisecond)",i, batVoltage, endRead - startRead);
        // if (batVoltage > 2.0)
        // {
        //   if (systemDefaultValue.runMode == 3)
        //     AD5940_Main(parameters); // for test 무한 루프
        // }
        // if(systemDefaultValue.runMode ==0) break;
        vTaskDelay(1000);
      }
    }
 //   globalModbusId = globalModbusId > 4 ? 1 : globalModbusId ;
    loopCount++;
    previous_5Secondmills = now;
  }
  if ((now - previous_30Secondmills > Interval_30Second))
  {
    previous_30Secondmills= now;
  }
  if ((now - previous_60Secondmills > Interval_60Second))
  {
    gettimeofday(&tmv, NULL);
    struct tm *timeinfo = gmtime(&tmv.tv_sec);
    simpleCli.outputStream->printf("\r\nEveryMinute reached  ... %d %d %d",timeinfo->tm_min,logForHour,timeinfo->tm_hour);
    if(logForHour != timeinfo->tm_hour){ //매시간마다 로그를 기록한다.
      logForHour = timeinfo->tm_hour;
      lsFile.writeCellDataLog();
    }
    // sendSelectBattery(impedanceCellPosition);//selecectedCellNumber를 변화 시킨다
    // AD5940_Main(parameters);  
    // impedanceCellPosition++;
    // if(impedanceCellPosition >= INSTALLED_CELLS)impedanceCellPosition =1;
    previous_60Secondmills= now;
  }
  vTaskDelay(100);
}