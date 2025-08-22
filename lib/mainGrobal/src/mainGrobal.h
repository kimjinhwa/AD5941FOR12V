#ifndef _MAIN_GROVAL_H
#define _MAIN_GROVAL_H
#include <Arduino.h>


#define SERIAL_RX1 26 // ok Serial1 통신 485모드버스통신으로 외부와의 인터페이스에 사용한다 
#define SERIAL_TX1 22 

#define IN_TH1 GPIO_NUM_34
#define IN_TH2 GPIO_NUM_35

#define EXT_485EN_1         GPIO_NUM_4  
#define RST_5941            GPIO_NUM_5  // 4951칩을 리셋하기 위함. 
#define PORT1               GPIO_NUM_19
#define PORT2               GPIO_NUM_18 
#define PORT3               GPIO_NUM_27 

#define SEL_ADD1            GPIO_NUM_33 
#define SEL_ADD2            GPIO_NUM_25 
#define SEL_ADD3            GPIO_NUM_23  
#define SEL_ADD4            GPIO_NUM_21  

#define READ_BATVOL         GPIO_NUM_36  //배터리 전압을 읽는다. 
#define MISO                GPIO_NUM_12  
#define MOSI                GPIO_NUM_13  
#define SCK                 GPIO_NUM_14  
                
#define AD5940_ISR          GPIO_NUM_32  
#define CS_5940             GPIO_NUM_15  

#define ESP_INTR_FLAG_DEFAULT 0
#define ESP_INTR_FLAG_DEFAULT 0

#define MAX_INSTALLED_CELLS 50

typedef struct
{
    char ssid[20];
    char ssid_password[10];
    char userid[10];
    char userpassword[10];                        
    uint8_t runMode; // 0: manual 1 : auto
    uint32_t IPADDRESS;   // 50 + 4 =54
    uint32_t GATEWAY;     // 54 + 4 = 58
    uint32_t SUBNETMASK;  // 58 + 4 = 62
    uint8_t modbusId;     // 62 + 1 = 63
    uint16_t installed_cells;     // 63 + 2 = 65
    uint16_t AlarmTemperature;    // 65 + 2 = 67
    uint16_t AlarmAmpere;    // 69
    uint16_t alarmHighCellVoltage;    // 71 
    uint16_t alarmLowCellVoltage;    // 73
    uint16_t cutoffHighCellVoltage;    // 75 
    uint16_t cutoffLowCellVoltage;    // 77
    uint16_t alarmDiffCellVoltage;    // 75 + 1 = 76
    int16_t voltageCompensation[40];// 76 + 80 =  156byte 
    int16_t impendanceCompensation[40];// 156 + 80 = 236
    float real_Cal;  // 236+4 = 240
    float image_Cal; // 240 + 4 = 248
    uint8_t logLevel; // 240 + 4 = 248
    uint16_t startBatnumber;     // 63 + 2 = 65
} nvsSystemSet;
extern nvsSystemSet systemDefaultValue;

typedef struct {
  time_t readTime; // 4byte
  float voltage;// 4byte
  float impendance;// 4byte
  int16_t temperature;// 2byte
  int16_t voltageCompensation;// 2byte
  int16_t impendanceCompensation;// 2byte
}_cell_value; // Total 18bte
extern _cell_value cellvalue[MAX_INSTALLED_CELLS];

typedef struct {
  uint16_t CellNo;
  time_t readTime; // 4byte
  float voltage;// 4byte
  int16_t temperature;// 2byte
  float impendance;// 4byte
}_cell_value_iv; // Total 18bte

typedef struct {
  time_t readTime; // 4byte
  float voltage[20];// 4byte
  float impendance[20];// 4byte
  int16_t temperature[20];// 2byte
}cell_logData_t; // Total 18bte

#endif