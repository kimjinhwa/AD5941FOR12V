#ifndef _MAIN_GROVAL_H
#define _MAIN_GROVAL_H
#include <Arduino.h>


#define SERIAL_RX1 26 // ok Serial1 통신 485모드버스통신으로 외부와의 인터페이스에 사용한다 
#define SERIAL_TX1 22 
#define SERIAL_RX2 18 // 확장된 Serial2로서 사용한다.
#define SERIAL_TX2 19

#define READ_BATVOL         36  //배터리 전압을 읽는다. 
#define SERIAL_SEL_ADDR1     2   // test OK 
#define SERIAL_SEL_ADDR0    23  // test OK Multi Serial의 선택을 한다.
#define EXT_485EN_1          4  // TEST OK 
#define RESET_N              5  // TEST OK 
#define RESET_5940           5  // 4951칩을 리셋하기 위함. 
#define MISO                12  // ok
#define MOSI                13  // ok
#define SCK                 14  // ok
#define CS_5940             15  // test ok
#define CELL485_DE          21  // 셀센서를 읽기 위한 485 DE 
#define EXT_P15_RELAY           27  //Ref를 읽는지 Battery를 읽는지 결정한다. 
                                // 0 :REF, 1: BAT  TEST OK
                                // TEST OK
#define AD5940_ISR          32  // OK  RTC 칩을 Enable한다.
#define RTC1305_EN          33 // OK  RTC 칩을 Enable한다.
#define LED_OP              25 // system operating 상태 

#define RELAY_FP_IO             0  // + 릴레이 
#define RELAY_FN_GND             25 // - 릴레이 

#define P15_MODE   1  // F+ 즉 셋서 리딩모드
#define SENSE_MODE    0  // P15V출력모드

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
// #define SERIAL_SEL_ADDR3    34  // Only Use Input
// #define SERIAL_SEL_ADDR2    35  // test fail 
// 2 OK 
// 0 0 : UART2 -> UU0 : 485통신을 하며 셀을 제어한다. 
// Modbus통신을 사용하며,  CELL485_DE 을 같이 사용한다.
// 0 1 : UART2 -> UU1 : Display
//    TxD, RxD CMOS TTL 통신을 하면 Display와 통신한다. 
// 1 0 : 외부장치와 통신을 하며  TTL레벨로서 
//    향후 이더넷 통신보드와 통신할때 사용한다.

// _sck = (_spi_num == VSPI) ? SCK : 14;
// _miso = (_spi_num == VSPI) ? MISO : 12;
// _mosi = (_spi_num == VSPI) ? MOSI : 13;
// _ss = (_spi_num == VSPI) ? SS : 15;

#endif