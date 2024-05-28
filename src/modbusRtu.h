#ifndef _MODBUSRTU_H
#define _MODBUSRTU_H
#include "ModbusServerRTU.h" 

#define TX2_PIN 12   // Serial2 RX 핀 (GPIO12)
#define RX2_PIN 14   // Serial2 TX 핀 (GPIO14)
ModbusMessage FC01(ModbusMessage request) ;
ModbusMessage FC03(ModbusMessage request) ;
ModbusMessage FC04(ModbusMessage request) ;
ModbusMessage FC05(ModbusMessage request) ;
ModbusMessage FC06(ModbusMessage request) ;

// //사용법
// //#include "modbusRtu.h"
// //전역
// //modbusRtu LcdCellRtu485;
// //setup
// // LcdCellRtu485.modbusInit();
// //  pinMode(OP_LED , OUTPUT);
// //loop
// //  LcdCellRtu485.receiveData();

// enum Error : uint8_t {
//   SUCCESS                = 0x00,
//   ILLEGAL_FUNCTION       = 0x01,
//   ILLEGAL_DATA_ADDRESS   = 0x02,
//   ILLEGAL_DATA_VALUE     = 0x03,
//   SERVER_DEVICE_FAILURE  = 0x04,
//   ACKNOWLEDGE            = 0x05,
//   SERVER_DEVICE_BUSY     = 0x06,
//   NEGATIVE_ACKNOWLEDGE   = 0x07,
//   MEMORY_PARITY_ERROR    = 0x08,
//   GATEWAY_PATH_UNAVAIL   = 0x0A,
//   GATEWAY_TARGET_NO_RESP = 0x0B,
//   TIMEOUT                = 0xE0,
//   INVALID_SERVER         = 0xE1,
//   CRC_ERROR              = 0xE2, // only for Modbus-RTU
//   FC_MISMATCH            = 0xE3,
//   SERVER_ID_MISMATCH     = 0xE4,
//   PACKET_LENGTH_ERROR    = 0xE5,
//   PARAMETER_COUNT_ERROR  = 0xE6,
//   PARAMETER_LIMIT_ERROR  = 0xE7,
//   REQUEST_QUEUE_FULL     = 0xE8,
//   ILLEGAL_IP_OR_PORT     = 0xE9,
//   IP_CONNECTION_FAILED   = 0xEA,
//   TCP_HEAD_MISMATCH      = 0xEB,
//   EMPTY_MESSAGE          = 0xEC,
//   ASCII_FRAME_ERR        = 0xED,
//   ASCII_CRC_ERR          = 0xEE,
//   ASCII_INVALID_CHAR     = 0xEF,
//   BROADCAST_ERROR        = 0xF0,
//   UNDEFINED_ERROR        = 0xFF  // otherwise uncovered communication error
// };
// class RTUutils {
// public:

//   static uint16_t calcCRC(const uint8_t *data, uint16_t len);
//   static bool validCRC(const uint8_t *data, uint16_t len);
//   static bool validCRC(const uint8_t *data, uint16_t len, uint16_t CRC);
// //   static uint32_t calculateInterval(uint32_t baudRate);
// //   inline static void RTSauto(bool level) { return; } // NOLINT

// protected:
//   RTUutils() = delete;

// };
// class modbusRtu 
// {
// private:
//     uint8_t _modBusID=1;
// public:
//     modbusRtu(){ };
//     void setModbusID(uint8_t modBusID){
//       modBusID = _modBusID;
//     };
//     void modbusInit();
//     void receiveData();
//     void F03(uint8_t *buffer,uint16_t len );
//     void F04(uint8_t *buffer,uint16_t len );
//     void F06(uint8_t *buffer,uint16_t len );
//     void errorPacket(uint8_t *buffer,Error errorCode);
// };
#endif