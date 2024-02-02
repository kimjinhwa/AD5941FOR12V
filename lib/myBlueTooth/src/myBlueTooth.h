#ifndef _MY_BLUE_TOOTH_H
#define  _MY_BLUE_TOOTH_H
#include <Arduino.h>
#include "SimpleCLI.h"
class myBlueTooth  {
    public :
    myBlueTooth();
    void readInputSerialBT();
    private:
    String input = "";
};

void blueToothTask(void *parameter);
#endif