#pragma once
#include <Arduino.h>
#include "mainGrobal.h"

class SelectCell {
  private:
  uint8_t currentPort=0;
    uint8_t port1=0;
    uint8_t port2=0;
    uint8_t port3=0;
    uint8_t port4=0;
    uint8_t port5=0;

  public:
  SelectCell(){
  }
  void select(uint8_t portNumber){
    // 0 : LED 1: Bat 1, 2: Bat 2, 3: Bat 3, 4: Bat 4, 5: Bat 5, 6: Bat 6, 7: Bat 7, 8: Bat 8, 9: Bat 9, 10: Bat 10
    currentPort = portNumber;
    digitalWrite(PORT1, LOW);
    digitalWrite(PORT2, LOW);
    digitalWrite(PORT3, LOW);
    digitalWrite(PORT4, LOW);
    digitalWrite(PORT5, LOW);
    delay(200);
    
    digitalWrite(PORT1, (0x01 & portNumber) ? HIGH : LOW);
    digitalWrite(PORT2, (0x02 & portNumber) ? HIGH : LOW);
    digitalWrite(PORT3, (0x04 & portNumber) ? HIGH : LOW);
    digitalWrite(PORT4, (0x08 & portNumber) ? HIGH : LOW);
    digitalWrite(PORT5, (0x10 & portNumber) ? HIGH : LOW);
    delay(200);
  }
  uint8_t getCurrentPort(){
    return currentPort;
  }
};
extern SelectCell selectCell;