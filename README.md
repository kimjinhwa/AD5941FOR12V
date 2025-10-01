## 12V BMS For 4 Cell
#VERSION "3.1.3"  
- 최대 셀수를 MAX_INSTALLED_CELLS로 제한한다.
- 현재 버전은 최대 20셀을 넘지 않게 한다. 
- 시스템 기본 변수 2개를 추가한다. 
```C
    int16_t baseVoltage[20];// 76 + 80 =  156byte 
    int16_t baseImpendance[20];// 156 + 80 = 236
```
#MODBUS ADDRESS 03,06
- 최대 요청수 : 256
- Address : 0~255



### Purpose
- To support GunNam-Dam
- Board Version ...
- Project Forder    
C:\Users\ServerManager\OneDrive\바탕 화면\1.PARACTICE\2.범용12V_BMS_재설계\MainCircuit
### Test Method 
- BlueTooth를 연결한다. 
- Bat calibration을 하기위하여 runmode를 0으로 실행한다. 
- r 0 를 실행하여 Bat연결을 해제 한다. -> LED가 꺼진다.   
  LED는 0을 입력할시 꺼지면 BAT 선택한경우 켜진다.

- r 1 를 실행하여 Bat연결한다.
- cal를 수행한다.
- 정상적으로 수행이 되는 것을 확인하면, 다시한번 cal save를 입력한다.
### Power Consumtion
- 500mA

