
#ifndef _MODBUSCELLMODULE_H
#define _MODBUSCELLMODULE_H
#include <Arduino.h> 
#include "mainGrobal.h"

bool sendSelectBatteryWithRetry(uint8_t modbusId);
uint16_t sendGetMoubusTemperature(uint8_t modbusId, uint8_t fCode);
typedef struct
{
    uint8_t relay1;
    uint8_t relay2;
}modbus_cellRelay_t ;

typedef struct
{
    uint16_t temperature;
    uint16_t modbusid;
    uint16_t baudrate;
}modbus_cellData_t ;


class ExtendSerial//: public HardwareSerial
{
  private:
    const uint8_t _addr0=23;
    const uint8_t _addr1=2;
    bool _addrValue0;
    bool _addrValue1;
  public:
    ExtendSerial(){
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
extern ExtendSerial extendSerial;
#endif