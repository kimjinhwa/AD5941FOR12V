#include "modbusLcdModule.h"
#include "modbusCellModule.h"
#include "ModbusClientRTU.h"
//#include "ModbusServerRTU.h"
#include "modbusRtu.h"
//#include "AD5940.h"


ModbusClientRTU modBusRtuLcd(-1,100);
static uint32_t request_response ;
static bool data_ready = false;
extern ExtendSerial extendSerial;

void handleDataLcd(ModbusMessage response, uint32_t token)
{
    uint16_t offs = 3;
    uint16_t *values;
    uint8_t func = response.getFunctionCode();

    if (func == WRITE_MULT_REGISTERS)
    {
        int address;
        response.getServerID();
        const uint8_t *rev = response.data();
    
        int len = response.size();
        address = rev[2] << 8 | rev[3];
        //ESP_LOGE("MODBUS", "multi func %d token %d address %d size = %d", func, token, address,len);
        request_response = token;
    }
}
void handleErrorLcd(Error error, uint32_t token) 
{
  ModbusError me(error);
  ESP_LOGE("MODBUS","Error response: %02X - %s\n", (int)me, (const char *)me);
}
void modbusLcdSetup()
{
  modBusRtuLcd.onDataHandler(&handleDataLcd);
  modBusRtuLcd.onErrorHandler(&handleErrorLcd);
  modBusRtuLcd.setTimeout(1000);
  modBusRtuLcd.begin(Serial2,BAUDRATESERIAL2,1);
}

void setSendbuffer(uint8_t fCode,uint16_t *sendValue);
Error setDataToLcd(uint16_t  startAddress, uint8_t sendLength)
{
    extendSerial.selectLcd();
    // Serial2.printf("0123456789\n");
    // Serial2.flush();
    uint16_t sendValue[256];
    memset(sendValue, 0x00, sizeof(sendValue));
    setSendbuffer(04,sendValue);
    uint32_t token = millis();
    uint8_t modbusId = 1;
    ModbusMessage rc;// = modBusRtuLcd.syncRequest(m,token);
    rc = modBusRtuLcd.syncRequest(token,1,WRITE_MULT_REGISTERS,startAddress,sendLength,sendLength*2,sendValue+startAddress);

        // token, modbusId, WRITE_MULT_REGISTERS /*fCode*/,
        // 0 /*startAddress*/, 255 /*quantity*/, 512 /*len*/,
        // (uint16_t *)&sendValue);
    if (rc.getError() == 0)
    {
        handleDataLcd(rc, token);
    }
    else
    {
        ESP_LOGW("MODUBS", "MODBUS ERROR %d", rc.getError());
        return rc.getError();
        token= 0;
    }
    return rc.getError();
}