
#ifndef _MODBUSCELLMODULE_H
#define _MODBUSCELLMODULE_H
#include <Arduino.h> 
#include "mainGrobal.h"
#include <Stream.h>  
#include <queue>
#include <mutex>
#include <vector>

using std::queue;

void modbusCellModuleSetup();
bool SelectBatteryMinusPlus(uint8_t modbusId);
uint32_t sendGetMoubusModuleData_old(uint8_t modbusId, uint8_t fCode);
uint32_t sendGetMoubusModuleData(uint32_t token,uint8_t modbusId, uint8_t fCode,uint16_t startAddress, uint16_t len);
typedef struct
{
    uint16_t modbusId;
    uint16_t funcCode;
    uint16_t address;
    uint16_t relay1;
    uint16_t relay2;
    uint16_t reserved_6;
    uint16_t reserved_7;
    uint16_t reserved_8;
    uint16_t reserved_9;
    uint16_t reserved_10;
    uint16_t reserved_11;
    uint16_t reserved_12;
    uint16_t reserved_13;
    uint16_t reserved_14;
    uint16_t reserved_15;
    uint16_t reserved_16;
}modbus_cellRelay_t ;

typedef struct
{
    uint16_t temperature;
    uint16_t modbusid;
    uint16_t baudrate;
    uint16_t reserved_4;
    uint16_t reserved_5;
    uint16_t reserved_6;
    uint16_t reserved_7;
    uint16_t reserved_8;
    uint16_t reserved_9;
    uint16_t reserved_10;
    uint16_t reserved_11;
    uint16_t reserved_12;
    uint16_t reserved_13;
    uint16_t reserved_14;
    uint16_t reserved_15;
    uint16_t reserved_16;
}modbus_cellData_t ;

typedef struct
{
    uint16_t modbusid;
    uint16_t funCode;
    uint16_t address;
    uint16_t data;
    uint16_t checksum;
}modbus_data_t ;

extern modbus_cellRelay_t modbusCellrelay;
extern modbus_cellData_t modbusCellData;
extern modbus_data_t writeHoldRegister;


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

class ModbusRequestModule
{
private:
  uint16_t MR_qLimit; // Maximum number of requests to hold in the queue
  std::mutex qLock;
  std::mutex countAccessM;
  uint32_t messageCount;
  struct RequestEntry
  {
    uint32_t token;
    uint16_t modbusID;
    uint16_t func;
    uint16_t address;
    uint16_t lendata;
    RequestEntry(uint32_t tok, uint16_t mod, uint16_t fun, uint16_t add, uint16_t len) : token(tok),
                                                                                         modbusID(mod),
                                                                                         func(fun),
                                                                                         address(add),
                                                                                         lendata(len)
    {
    }
  };

public:
  RequestEntry reqEntry;
  queue<RequestEntry> requestModuleQueue;
  ModbusRequestModule(uint16_t queueLimit = 10) : MR_qLimit(queueLimit), reqEntry(0,0,0,0,0),messageCount(0)
  {
    MR_qLimit = queueLimit;
  }
  bool addToQueue(uint32_t tok, uint16_t mod, uint16_t fun, uint16_t add, uint16_t len)
  {
    RequestEntry entry(tok, mod, fun, add, len);
    bool rc = false;
    if (requestModuleQueue.size() < MR_qLimit)
    {
      rc = true;
      std::lock_guard<std::mutex> lockGuard(qLock);
      requestModuleQueue.push(entry);
    }
    std::lock_guard<std::mutex> countLock(countAccessM);
    return rc;
  };
  void pop()
  {
    std::lock_guard<std::mutex> lockGuard(qLock);
    if(!requestModuleQueue.empty()){
      reqEntry = requestModuleQueue.front();
      requestModuleQueue.pop();
    }
  }
};
extern ModbusRequestModule modbusRequestModule;
#endif