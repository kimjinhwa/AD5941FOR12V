#ifndef _MY_BLUE_TOOTH_H
#define  _MY_BLUE_TOOTH_H
#include <Arduino.h>
#include "SimpleCLI.h"
#include "simpleBLE.h"

class myBlueTooth  {
    public :
    myBlueTooth();
    void readInputSerialBT();
    void initBLE();
    void sendBLEData(String data);
    bool isBLEConnected();
    void setBLEDataCallback(void (*callback)(String));
    void setBLEConnectionCallback(void (*callback)(bool));
    void startAdvertising();
    void stopAdvertising();
    private:
    String input = "";
    SimpleBLE* bleServer;
};

void blueToothTask(void *parameter);
void bleServerTask(void *parameter);
#endif