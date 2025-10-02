## 12V BMS For 4 Cell
#define VERSIOn "3.1.6" 임피던스 계산공식을 위하여 시스템메모리변수를 추가한다.
```C
  sendValue[120]=systemDefaultValue.real_Cal;
  sendValue[121]=systemDefaultValue.image_Cal;
  sendValue[160] = systemDefaultValue.ImpedanceFactor;
  sendValue[161] = systemDefaultValue.VoltageFactor;
  sendValue[162] = systemDefaultValue.TemperatureFactor;
```
#VERSION "3.1.5" // 임피던스는 하루에 한번만 읽기를 수행한다.
- 다른시간에는 전압읽기를 하며 이때 ad5941는 RUN을 중지한다. 이것은 부하가 걸릴때 SSR의 전압드롭을 방지 하기 위함이다.
- 처음 시스템이 시작될때는 메모리에 있는 전압과 임피턴스를 리턴한다.
- 모드버스를 이용하여 즉시 임피던스 측정모드로 진입하게 한다.


#VERSION "3.1.3" //모드버스 아이디를 변경하면 기본 하드코딩된 값으로 baseVoltage, baseImpendance
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
- 0~20 : voltageCompensation
- 20~40 : baseVoltage 
- 40~80 : Temperature
- 80~100 : ImpandanceCompensatirion.
- 100~120 : BaseImpedance 
```C
  sendValue[126]= systemDefaultValue.modbusId ;
  sendValue[127]= systemDefaultValue.installed_cells;
  sendValue[128]= systemDefaultValue.AlarmTemperature;
  sendValue[129]= systemDefaultValue.alarmHighCellVoltage ;
  sendValue[130]=systemDefaultValue.alarmLowCellVoltage;
  sendValue[131]= systemDefaultValue.AlarmAmpere ;  // 200A

  sendValue[141] = systemDefaultValue.ACVoltPP;
  sendValue[142] = systemDefaultValue.DCVolt;
  sendValue[143] = systemDefaultValue.SinFreq;
  sendValue[144] = systemDefaultValue.RcalLoopCount;
  sendValue[145] = selectCell.getCurrentPort();
```




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

