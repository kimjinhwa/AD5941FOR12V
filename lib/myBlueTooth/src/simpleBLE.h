#ifndef _SIMPLE_BLE_H
#define _SIMPLE_BLE_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE 서비스 및 특성 UUID 정의
#define SERVICE_UUID1        "12345678-1234-1234-1234-123456789ABC"
#define CHARACTERISTIC_UUID_RX1 "12345678-1234-1234-1234-123456789ABD"
#define CHARACTERISTIC_UUID_TX1 "12345678-1234-1234-1234-123456789ABE"

#define SERVICE_UUID2        "22345678-1234-1234-1234-123456789ABC"
#define CHARACTERISTIC_UUID_RX2 "22345678-1234-1234-1234-123456789ABD"
#define CHARACTERISTIC_UUID_TX2 "22345678-1234-1234-1234-123456789ABE"

class SimpleBLE {
public:
    SimpleBLE(int modbusId);
    ~SimpleBLE();
    
    // BLE 서버 초기화 및 시작
    void initServer(String deviceName);
    void startAdvertising();
    void stopAdvertising();
    
    // 연결 상태 확인
    bool isConnected();
    
    // 데이터 송수신
    void sendData(String data);
    void setDataCallback(void (*callback)(String));
    
    // 연결 상태 콜백
    void setConnectionCallback(void (*callback)(bool));
    
private:
    BLEServer* pServer;
    BLECharacteristic* pCharacteristicTX;
    BLECharacteristic* pCharacteristicRX;
    BLEUUID SERVICE_UUID;
    BLEUUID CHARACTERISTIC_UUID_TX;
    BLEUUID CHARACTERISTIC_UUID_RX;
    
    bool deviceConnected;
    bool oldDeviceConnected;
    
    
    void (*dataCallback)(String);
    void (*connectionCallback)(bool);
    
    // 내부 콜백 클래스들
    class MyServerCallbacks : public BLEServerCallbacks {
    public:
        MyServerCallbacks(SimpleBLE* parent) : parent(parent) {}
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;
    private:
        SimpleBLE* parent;
    };
    
    class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    public:
        MyCharacteristicCallbacks(SimpleBLE* parent) : parent(parent) {}
        void onWrite(BLECharacteristic *pCharacteristic) override;
    private:
        SimpleBLE* parent;
    };
    
    MyServerCallbacks* serverCallbacks;
    MyCharacteristicCallbacks* characteristicCallbacks;
};

#endif
