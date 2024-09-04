#include "modbusLcdModule.h"

//#include "ModbusClientRTU.h"
#include "ModbusServerRTU.h"
//#include "modbusRtu.h"
//#include "AD5940.h"


ModbusServerRTU LcdCell485(2000,CELL485_DE);// LCD를 위하여 사용한다.

ModbusMessage FC03(ModbusMessage request) ;
ModbusMessage FC04(ModbusMessage request) ;
ModbusMessage FC06(ModbusMessage request);
void setupModbusAgentForLcd()
{
  uint8_t address_485 = systemDefaultValue.modbusId; 
  LcdCell485.useStopControll =1;
  LcdCell485.begin(Serial2,BAUDRATESERIAL2,1);
  LcdCell485.registerWorker(address_485,READ_HOLD_REGISTER,&FC03);
  LcdCell485.registerWorker(address_485,READ_INPUT_REGISTER,&FC04);
  LcdCell485.registerWorker(address_485,WRITE_HOLD_REGISTER,&FC06);
}