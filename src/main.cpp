#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <wifi.h>
#include <driver/adc.h>
#include "filesystem.h"
#include "maingrobal.h"
#include "SimpleCLI.h"

#include "stdio.h"
#include "AD5940.h"
#include "HardwareSerial.h"
#include <BluetoothSerial.h>
#include "myBlueTooth.h"
#include "NetworkTask.h"
#include "ModbusClientRTU.h"
#include "modbusRtu.h"
#include "batDeviceInterface.h"
#include "mainClass.hpp"

#include <esp_task_wdt.h>

#define MAIN_POWEROFF HIGH
#define MAIN_POWERON LOW 
#define WDT_TIMEOUT 100 
// 기본 vSPI와 일치한다
#define VSPI_MISO   MISO  // IO19
#define VSPI_MOSI   MOSI  // IO 23
#define VSPI_SCLK   SCK   // IO 18
#define VSPI_SS     15    // IO 15

#define NUM_VALUES 21

static const int spiClk = 1000000; // 1 MHz
static char TAG[] ="Main";

TaskHandle_t *h_pxblueToothTask;
TaskHandle_t *h_pxNetworkTask;
nvsSystemSet systemDefaultValue;

ModbusServerRTU external485(2000,EXT_485EN_1);

uint32_t request_time;
uint16_t values[2];
uint16_t cellModbusIdReceived;

uint8_t selecectedCellNumber =0;

_cell_value cellvalue[MAX_INSTALLED_CELLS];

extern SimpleCLI simpleCli;
uint16_t startBatnumber=1;

BluetoothSerial SerialBT;

BatDeviceInterface batDevice;

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
uint8_t impedanceCellPosition=1;
static timeval tmv;
int16_t logForHour=0;
uint32_t loopCount=0;
static bool isModuleBootingOK=false;
static long elaspTime=-1;
SelectCell selectCell;

void AD5940_ShutDown();

void setErrorMessageToModbus(bool setError,const char* msg);

void pinsetup()
{
    pinMode(READ_BATVOL, INPUT);
    pinMode(AD5940_ISR, INPUT);

    pinMode(EXT_485EN_1, OUTPUT);
    digitalWrite(EXT_485EN_1, LOW);
    pinMode(RST_5941, OUTPUT);
    digitalWrite(RST_5941, HIGH);
    pinMode(PORT1, OUTPUT);
    pinMode(PORT2, OUTPUT);
    pinMode(PORT3, OUTPUT);
    digitalWrite(PORT1, LOW);
    digitalWrite(PORT2, LOW);
    digitalWrite(PORT3, LOW);
    pinMode(SEL_ADD1, OUTPUT);
    pinMode(SEL_ADD2, OUTPUT);
    pinMode(SEL_ADD3, OUTPUT);
    pinMode(SEL_ADD4, INPUT);
    digitalWrite(SEL_ADD1, HIGH);
    digitalWrite(SEL_ADD2, HIGH);
    digitalWrite(SEL_ADD3, HIGH);
    digitalWrite(SEL_ADD4, HIGH);

    pinMode(CS_5940, OUTPUT);
    digitalWrite(CS_5940, HIGH);
    pinMode(RST_5941, OUTPUT);
    digitalWrite(RST_5941, HIGH);

    pinMode(SEL_ADD1, OUTPUT);
    pinMode(SEL_ADD2, OUTPUT);
    pinMode(SEL_ADD3, OUTPUT);
    pinMode(SEL_ADD4, INPUT);
}


//HardwareSerial Serial1;
void AD5940_Main(void *parameters);

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
    systemDefaultValue.installed_cells= 20;
    strncpy(systemDefaultValue.userid,"admin",10);
    strncpy(systemDefaultValue.userpassword,"admin",10);
    for(int i=0;i<40;i++){
      systemDefaultValue.voltageCompensation[i]=0;
      systemDefaultValue.impendanceCompensation[i]=0;
    }
    systemDefaultValue.real_Cal = -33410.0f;
    systemDefaultValue.image_Cal = 35511.0f;
    systemDefaultValue.logLevel = ESP_LOG_INFO;
    systemDefaultValue.startBatnumber = 1;
    EEPROM.writeByte(0, 0x55);
    EEPROM.writeBytes(1, (const byte *)&systemDefaultValue, sizeof(nvsSystemSet));
    EEPROM.commit();
  }
  EEPROM.readBytes(1, (byte *)&systemDefaultValue, sizeof(nvsSystemSet));
  if(systemDefaultValue.startBatnumber > systemDefaultValue.installed_cells  )
    systemDefaultValue.startBatnumber = systemDefaultValue.installed_cells;
  if(systemDefaultValue.startBatnumber == 0) 
  startBatnumber = 1;
}

  

void setModbusAgent(){
  //address는 항상 1이다.
  uint8_t address_485 = systemDefaultValue.modbusId; 
  RTUutils::prepareHardwareSerial(Serial1);
  Serial1.begin(BAUDRATESERIAL1, SERIAL_8N1, SERIAL_RX1, SERIAL_TX1);
  //external485.useStopControll =0;
  external485.begin(Serial1,BAUDRATE,1,2000);
  external485.registerWorker(address_485,READ_COIL,&FC01);
  external485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  external485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  external485.registerWorker(address_485,WRITE_COIL,&FC05);
  external485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);

};


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
// 인터럽트 서비스 루틴 (ISR)
// void IRAM_ATTR handleInterrupt() {
//   // 인터럽트가 발생했을 때 실행될 코드
//   Serial.println("Interrupt detected!");
// }
String bootingReasonCheck()
{
  esp_reset_reason_t resetReson = esp_reset_reason();

  String strReset[]{
      // ESP_RST_UNKNOWN:
      " Reset reason can not be determined",
      // case ESP_RST_POWERON:
      " Reset due to power-on event",
      // case ESP_RST_EXT:
      " Reset by external pin (not applicable for ESP32)",
      // case ESP_RST_SW:
      " Software reset via esp_restart",
      // case ESP_RST_PANIC:
      " Software reset due to exception/panic",
      // case ESP_RST_INT_WDT:
      " Reset (software or hardware) due to interrupt watchdog",
      // case ESP_RST_TASK_WDT:
      " Reset due to task watchdog",
      // case ESP_RST_WDT:
      " Reset due to other watchdogs",
      // case ESP_RST_DEEPSLEEP:
      " Reset after exiting deep sleep mode",
      // case ESP_RST_BROWNOUT:
      " Brownout reset (software or hardware)",
      // case ESP_RST_SDIO:
      " Reset over SDIO",
  };

  String strReason = strReset[resetReson];
  return strReason;
}
void setup()
{

  Serial.begin(BAUDRATE);
  Serial.println("System booting....");
  EEPROM.begin(sizeof(nvsSystemSet) + 1);
  Serial.println("EEPROM begin....");
  readnWriteEEProm();
  Serial.println("EEPROM read....");
  pinsetup();
  Serial.println("pinsetup....");

  String strResetReason = "System booting reason is  ";
  strResetReason += bootingReasonCheck();
  Serial.println(strResetReason);
  memset(cellvalue,0,sizeof(cellvalue));
  Serial.println("ModbusAgent set....");
  setModbusAgent();
  String bleName = "TIMP_";
  String WifiAddress = WiFi.macAddress();
  bleName += WifiAddress;
  bleName += "_";
  bleName += systemDefaultValue.modbusId;
  SerialBT.begin(bleName.c_str());
  Serial.printf("\nBluetooth Name : %s\n",bleName.c_str());
  long sTime = millis();
  Serial.println("LittleFS init....");
  lsFile.littleFsInitFast(0);
  Serial.printf("LittleFS init done in %ld miliseconds\n",millis()-sTime);

  SPI.setFrequency(spiClk);
  SPI.begin(SCK, MISO, MOSI, CS_5940);
  pinMode(SS, OUTPUT); // VSPI SS -> 아니다..이것은 리셋용이다.

  ESP_LOGI(TAG, "System Started at %s mode", systemDefaultValue.runMode == 0 ? "Manual" : "Auto");
  ESP_LOGI(TAG, "\nEEPROM installed Bat number %d", systemDefaultValue.installed_cells);
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
// #ifdef WEBOTA
//   xTaskCreate(NetworkTask, "NetworkTask", 5000, NULL, 1, h_pxNetworkTask); // PCB 패턴문제로 사용하지 않는다.
// #endif

  Serial.println("BlueTooth Task create....");
  xTaskCreate(blueToothTask, "blueToothTask", 5000, NULL, 1, h_pxblueToothTask);
  xTaskCreate(AD5940_Main, "AD5940_Main", 5000, NULL, 1, NULL);
  simpleCli.outputStream = &Serial;
  memset(cellvalue,0,sizeof(cellvalue));
};
void loop(void)
{
  bool bRet;
  void *parameters;
  esp_log_level_set("*",ESP_LOG_INFO);
  //parameters = simpleCli.outputStream;
  parameters = &Serial;
  now = millis(); 

  esp_task_wdt_reset();
  if ((now - previousSecondmills > everySecondInterval))
  {
    //if(elaspTime != -1) 
    elaspTime++;
    if( elaspTime%10 ==0 ) simpleCli.outputStream->printf("\nTime elasped : %d",elaspTime);
    uint8_t portNumber = elaspTime%2;
    simpleCli.outputStream->printf("\nPort1 : %d, Port2 : %d, Port3 : %d portNumber : %d",
      digitalRead(PORT1),digitalRead(PORT2),digitalRead(PORT3),portNumber);
    // esp_task_wdt_reset();
    selectCell.select(portNumber);
    // delay(100);
    previousSecondmills = now;
  }
  if ((now - previous_3Secondmills > Interval_3Second))
  {
    previous_3Secondmills= now;
  }
  // if ((now - previous_5Secondmills > Interval_5Second) && (elaspTime % 60 ==0))
  // {
  //   if ((systemDefaultValue.runMode != 0) )  // 자동 모드에서만 실행한다
  //                                         // 또한 매 분 실행한다.
  //   {
  //     for (int i = startBatnumber; i <= systemDefaultValue.installed_cells; i++)
  //     {
  //       time_t startRead = millis();
  //       time_t endTime ;
  //       if(elaspTime==0) //처음으로 실행하는 것이면 
  //         elaspTime=3590; //이렇게 해서 처음에는 전압만 읽고
  //                         //다시 임피던스를 읽는 방식으로 하자.
  //       parameters = simpleCli.outputStream;
  //       selecectedCellNumber = i-1;
  //       (millis(), i, READ_INPUT_REGISTER, 0, 3,5); // 40mills
  //       endTime = millis();             // take 300ms
  //       //ESP_LOGI("TIME", "Elasp time Step 1 : %ld milisecond", endTime - startRead);
  //       esp_task_wdt_reset();
  //       //TODO: 아래의 펑션은 MODBUS 루틴을 변경하기 위해 임시로 막는다
  //       endTime = millis();             // take 300ms
  //       float batVoltage = 0.0;
  //       batVoltage = batDevice.readBatAdcValue(i, 600);
  //       if (batVoltage > 18.0)
  //         batVoltage = 0.0;
  //       cellvalue[i - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
  //       endTime = millis();             // take 300ms
  //       simpleCli.outputStream->printf("\ntime:%ld Bat(%i) Vol:%3.3f (%ldmili)\n",
  //         elaspTime,i, batVoltage, endTime - startRead);
  //       if (batVoltage > 2.0)
  //       {
  //         // 4는 cheating mode이다.
  //         if (systemDefaultValue.runMode >= 3 && (elaspTime%3600==0)) //매시간마다 실행한다
  //         {
  //           AD5940_Main(parameters); 
  //         }
  //       }
  //       if(systemDefaultValue.runMode ==0) break;
  //     }
  //   }
  //   loopCount++;
  //   previous_5Secondmills = millis();
  // }
  if ((now - previous_30Secondmills > Interval_30Second))
  {
    previous_30Secondmills= now;
  }
  if ((now - previous_60Secondmills > Interval_60Second))
  {
    previous_60Secondmills= now;
  }
  vTaskDelay(1000);
}