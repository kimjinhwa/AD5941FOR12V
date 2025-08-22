#pragma once
#include <Arduino.h>
#include "mainGrobal.h"

class SelectCell {
  private:
    uint8_t port1=0;
    uint8_t port2=0;
    uint8_t port3=0;

  public:
  SelectCell(){
  }
  void select(uint8_t portNumber){
    // 0 : LED 1: Bat 1, 2: Bat 2, 3: Bat 3, 4: Bat 4, 5: Bat 5, 6: Bat 6, 7: Bat 7, 8: Bat 8, 9: Bat 9, 10: Bat 10
    port1 = 0x01 & portNumber?HIGH:LOW;
    port2 = 0x02 & portNumber?HIGH:LOW;
    port3 = 0x04 & portNumber?HIGH:LOW;
    digitalWrite(PORT1,port1);
    digitalWrite(PORT2,port2);
    digitalWrite(PORT3,port3);
  }
};
extern SelectCell selectCell;