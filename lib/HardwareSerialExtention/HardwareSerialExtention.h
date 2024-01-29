
#ifndef HardwareSerialExtension_h
#define HardwareSerialExtension_h

#include <inttypes.h>
#include <functional>
#include "Stream.h"
#include "esp32-hal.h"
#include "soc/soc_caps.h"
#include "HWCDC.h"
//#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "HardwareSerial.h"

#define SERIAL_SEL_ADDR0    23  // Multi Serial의 선택을 한다.
#define SERIAL_SEL_ADDR1    2 

typedef enum {
    CELL485_PORT,
    DISPLAY_PORT,
    EXTERNAL_PORT
} hwExt_port_t;

#define SEL_CELL()   do{ digitalWrite(SERIAL_SEL_ADDR0    ,LOW);digitalWrite(SERIAL_SEL_ADDR1,LOW);}while(0)
#define SEL_DISPLAY() do{ digitalWrite(SERIAL_SEL_ADDR0    ,LOW);digitalWrite(SERIAL_SEL_ADDR1,HIGH);}while(0)
#define SEL_EXTERNAL() do{ digitalWrite(SERIAL_SEL_ADDR0    ,HIGH);digitalWrite(SERIAL_SEL_ADDR1,LOW);}while(0)

class HardwareSerialExtension : public HardwareSerial {
    public:
    HardwareSerialExtension(int uart_nr) : HardwareSerial(uart_nr){};
    void SetOutputPort(hwExt_port_t extPort = CELL485_PORT){
        this->extPort = extPort; 
        if(extPort == CELL485_PORT) SEL_CELL();
        else if(extPort == DISPLAY_PORT) SEL_DISPLAY();
        else if(extPort == EXTERNAL_PORT) SEL_EXTERNAL();
    }
    void begin(unsigned long baud,int rs485en_pin=4, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = 112)
    {   
        pinMode(rs485en_pin,OUTPUT);
        pinMode(SERIAL_SEL_ADDR0    ,OUTPUT);
        pinMode(SERIAL_SEL_ADDR1    ,OUTPUT);
        this->rs485en_pin= rs485en_pin;
        digitalWrite(rs485en_pin,LOW);
        HardwareSerial::begin(baud, config, rxPin, txPin, invert, timeout_ms , rxfifo_full_thrhd );
    }
    size_t write(uint8_t c) override 
    {
        if(extPort == CELL485_PORT) SEL_CELL();
        else if(extPort == DISPLAY_PORT) SEL_DISPLAY();
        else if(extPort == EXTERNAL_PORT) SEL_EXTERNAL();

        digitalWrite(rs485en_pin,HIGH);
        size_t result = HardwareSerial::write(c);
        flush();
        digitalWrite(rs485en_pin,LOW);
        return result;
    }
    private:
    int rs485en_pin;
    hwExt_port_t extPort;
};


#endif