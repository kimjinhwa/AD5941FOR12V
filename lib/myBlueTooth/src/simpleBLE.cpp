#include "simpleBLE.h"

// 생성자
SimpleBLE::SimpleBLE() {
    pServer = nullptr;
    pCharacteristicTX = nullptr;
    pCharacteristicRX = nullptr;
    deviceConnected = false;
    oldDeviceConnected = false;
    dataCallback = nullptr;
    connectionCallback = nullptr;
    serverCallbacks = nullptr;
    characteristicCallbacks = nullptr;
}

// 소멸자
SimpleBLE::~SimpleBLE() {
    if (pServer != nullptr) {
        pServer->getAdvertising()->stop();
    }
    if (serverCallbacks != nullptr) {
        delete serverCallbacks;
    }
    if (characteristicCallbacks != nullptr) {
        delete characteristicCallbacks;
    }
}

// BLE 서버 초기화
void SimpleBLE::initServer(String deviceName) {
    // BLE 디바이스 초기화
    BLEDevice::init(deviceName.c_str());
    
    // 보안 설정 (연결 문제 해결을 위해 완화)
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    
    // 서버 생성
    pServer = BLEDevice::createServer();
    
    // 콜백 객체 생성
    serverCallbacks = new MyServerCallbacks(this);
    pServer->setCallbacks(serverCallbacks);
    
    // 서비스 생성
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    // TX 특성 생성 (서버 -> 클라이언트)
    pCharacteristicTX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristicTX->addDescriptor(new BLE2902());
    
    // RX 특성 생성 (클라이언트 -> 서버) - 보안 설정 추가
    pCharacteristicRX = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    
    // RX 특성 콜백 설정
    characteristicCallbacks = new MyCharacteristicCallbacks(this);
    pCharacteristicRX->setCallbacks(characteristicCallbacks);
    
    // 서비스 시작
    pService->start();
    
    Serial.println("BLE Server initialized: " + deviceName);
    Serial.println("Service UUID: " + String(SERVICE_UUID));
    Serial.println("TX Characteristic UUID: " + String(CHARACTERISTIC_UUID_TX));
    Serial.println("RX Characteristic UUID: " + String(CHARACTERISTIC_UUID_RX));
}

// 광고 시작
void SimpleBLE::startAdvertising() {
    if (pServer != nullptr) {
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        
        // 서비스 UUID를 광고 데이터에 추가 (완전한 UUID)
        pAdvertising->addServiceUUID(SERVICE_UUID);
        
        // 스캔 응답 설정
        pAdvertising->setScanResponse(true);
        
        // 광고 설정 (연결 가능) - 더 호환성 좋은 설정
        pAdvertising->setMinPreferred(0x06);  // iPhone 연결 속도
        pAdvertising->setMaxPreferred(0x12);
        
        // 광고 간격 설정 (연결 문제 해결을 위해 조정)
        pAdvertising->setMinInterval(0x100);    // 160ms (더 안정적)
        pAdvertising->setMaxInterval(0x200);    // 320ms
        
        // 광고 타입 설정 (연결 가능한 광고)
        pAdvertising->setAdvertisementType(ADV_TYPE_IND);
        
        // 광고 시작
        pServer->getAdvertising()->start();
        
        Serial.println("=== BLE Server Advertising Started ===");
        Serial.println("Service UUID: " + String(SERVICE_UUID));
        Serial.print("Device Address: ");
        Serial.println(BLEDevice::getAddress().toString().c_str());
        Serial.println("Advertising Type: ADV_TYPE_IND");
        Serial.println("Min Interval: 160ms, Max Interval: 320ms");
        Serial.println("Power Level: ESP_PWR_LVL_P9");
        Serial.println("=====================================");
    }
}

// 광고 중지
void SimpleBLE::stopAdvertising() {
    if (pServer != nullptr) {
        pServer->getAdvertising()->stop();
        Serial.println("BLE Server advertising stopped");
    }
}

// 연결 상태 확인
bool SimpleBLE::isConnected() {
    return deviceConnected;
}

// 데이터 전송
void SimpleBLE::sendData(String data) {
    if (pCharacteristicTX != nullptr && deviceConnected) {
        pCharacteristicTX->setValue(data.c_str());
        pCharacteristicTX->notify();
        Serial.println("BLE Sent: " + data);
    } else {
        Serial.println("BLE Send failed - not connected");
    }
}

// 데이터 콜백 설정
void SimpleBLE::setDataCallback(void (*callback)(String)) {
    dataCallback = callback;
}

// 연결 상태 콜백 설정
void SimpleBLE::setConnectionCallback(void (*callback)(bool)) {
    connectionCallback = callback;
}

// 서버 콜백 구현
void SimpleBLE::MyServerCallbacks::onConnect(BLEServer* pServer) {
    parent->deviceConnected = true;
    Serial.println("BLE Client connected");
    if (parent->connectionCallback != nullptr) {
        parent->connectionCallback(true);
    }
}

void SimpleBLE::MyServerCallbacks::onDisconnect(BLEServer* pServer) {
    parent->deviceConnected = false;
    Serial.println("BLE Client disconnected");
    if (parent->connectionCallback != nullptr) {
        parent->connectionCallback(false);
    }
}

// 특성 콜백 구현
void SimpleBLE::MyCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = String(pCharacteristic->getValue().c_str());
    if (rxValue.length() > 0) {
        Serial.println("BLE Received: " + rxValue);
        if (parent->dataCallback != nullptr) {
            parent->dataCallback(rxValue);
        }
    }
}
