# TIA Resistance Measurement System.
## 시스템설정
  * 초기 전원을 ON할때는 반드시 센서모듈 라인을 분리한 후    
    토글 스위치를 OFF모드로 놓은 후 파워를 공급한다.
  * runmode  
    - 0 : Nothing
    - 1 : Read Voltage 
    - 2 : Not Defined 
    - 3 : Read Impedance 
    - 4 : Impedance Cheating Mode. 
  * bat <num>
    - 설치된 축전지 수를 설정한다. 
    - 자동으로 시스템은 재 부팅된다.
  * id <num>
    - modbus id 를 설정한다. 
  * offset [i][ia][v][va][vv] <num> <value>
     - -ia : 모든 셀에 대하여 공통적용한다.
     - -iv : num을 0으로 주면 모든 셀에 대하여 공통적용한다.
     - offset 0 0 현재의 설정된 값을 보여준다.
       대략적으로 1 스텝에 15mv로 본다.   
       offset 0 "-36"  와 같이 입력한다
  * cal : calibration 
     - 내부의 고정된 저항값으로 기준값을 읽는다.
     - cal save : 값을 측정하고 EEPROM에 저장한다. 한번 했다면 기존의 값을 활용하여 사용한다.
  * time : 시간 설정
    - time -y 2024 -mo 9 -d 12 -h 09 -m 47 -s 00 
  * reboot : 시스템을 재 시작한다.
  * p15relay : 메인보드에서 센서보드에 공급하는 신호 15V의 제어를 한다. 
    - p15relay 00 :  센서 보드의 두 라인 모두 P15를 출력한다.   
     이것은 센서모듈의 부팅 모드이다. 즉 이상태에서 라인을 꽃으면 정상적으로 센서 모듈이 부팅된다. 
  - p15relay 01 : 
  - p15relay 10 : 
  - p15relay 11 : 
* mod/uleid : 모듈의 ID를 노트북 없이 변경하고자 할 때 사용할 수 있다. 
  - 모듈 케이블을 본체에서 분리한다.
  - runmode 0 으로 놓고 시스템을 재 부팅한다.
  - p15relay 00 로 해서 센서 부팅라인에 전원을 공급하고나    
    케이스를 열고 점퍼를 뺀 후  케이블에 하나의 라인만 연결한다. 
  
  - mod <number> 로 아이디를 변경한다.
  - mod 를 입력하여 현재의 id를 확인한다.  ## 에러 유형 

  *  checkVoltageoff(): [Voltage] Bat OFF Check Voltage is : 18.994 
## BOARD DEBUGGING
  * WIFI 안테나 부분을 판다.
  * R1  100K를 200K로 .
  * P15V출력에 Toggle Switch를 달아 준다.
  * ADG656_SEL IO27 및 점퍼를 사용하지 않고 GPIO2_EXT로 확정한다 ( J2 삭제)
  * SENSE_P 과 SENSE_N을 바꿔준다.(회로도에서.. 상관은 없으나 이렇게 해주는게 좋다)
  *  adc_filter.Sinc2NotchEnable = bTRUE; 이것이 활성화 되어 있지 않다.    
     다시 확인을 하자. 
## BLUETOOTH COMMAND
### offset #
 #### offset    [-ia cellno value ] [-va  cellno value ]    [-i  cellno value ] 
 * [-ia cellno value ] 임피던스에 대한 옵셋을 맞춘다. 
   * cellno : 셀번호 1번부터시작한다.   
     0 이 주어지면 모든셀에 대하여 적용한다.   
   * value    
         systemDefaultValue.impendanceCompensation[i]의 
         값에 주어진 값을 += 한다.   
         ```
          cellvalue[i].impendance = cellvalue[i].impendance +   systemDefaultValue.impendanceCompensation[i] / 100.f;   
          ```
      따라서 Value는 1/100로 적용하기 때문에 100을 곱한 
      값으로 준다.   
 * [-va  cellno value ]  전압에 대한 옵셋을 맞춘다. 
      * cellno : 셀번호 1번부터시작한다.   
                  0 이 주어지면 모든셀에 대하여 적용한다.   
      * value    
         systemDefaultValue.voltageCompensation[i]의 
         값에 주어진 값을 += 한다.     
        ``` 
         systemDefaultValue.voltageCompensation[i] += value 
        ```
      따라서 Value는 1/100로 적용하기 때문에 100을 곱한 
      값으로 준다.   
 * [-i cellno value ] ia와 동일하나 +=이 아닌 =를 사용한다.
      ```
      cellvalue[i].impendance = cellvalue[i].impendance + systemDefaultValue.impendanceCompensation[i] / 100.f;
      ```
 * [-v cellno value ] va와 동일하나 +=이 아닌 =를 사용한다.
      ```
        systemDefaultValue.voltageCompensation[i] = value; // cellvalue[0].voltage - cellvalue[i].voltage;
        cellvalue[number - 1].voltage= cellvalue[number - 1].voltage+ systemDefaultValue.voltageCompensation[number - 1];
      ```

 * [-vv cellno value ] 입력한 값으로 옵셋을 맞추어 주는 옵션이다.     
 계산을 쉽하기 해 주기 위한 방법이다.   
 첫번째는 셀번호를 두번째는 원하는 전압값을 기록한다
 ```
    float gapVoltage = fvalue - cellvalue[number-1].voltage ;
    int voloffset = (int)(gapVoltage /0.005);
    systemDefaultValue.voltageCompensation[number - 1] = voloffset;
    //voltageCompensation값은 아래와 같이 adcReading Offset에 사용된다.
    systemDefaultValue.voltageCompensation[number - 1] = voloffset;
  uint32_t voltage = esp_adc_cal_raw_to_voltage((uint32_t)batVoltageAdcValue , &adc_chars);
 ``` 

# relay -s [Cell no] -off 
  - relay -s [Cell no] -off 
    - off : check if all cell is offed 
    - Cell no : 0 All Off, 
      number : Set Relay On the number and +1 relay on 
  - cal/ibration 기준 임피던스를 구한다. 
    cal save : 값을 읽고 저장한다.
    cal save 캘리브레이션 된 값을 EEPROM에 저장한다
  - mode  자동으로 임피던스를 읽을 것인지 아닌지를 결정한다. 
          Calibration을 수행하기 위해서는 이것이 수동으로 설정 되어 있어야 한다.
    - mode 0 :수동
    - mode 1 : 전압만 자동
    - mode 3 : 전압과 임피던스 자동 
  - bat/number [40]: 설치되어있는 배터리숫자를 기록한다
  - temp/erature : 전체의 온도를 읽는 루틴이다.
  - time : 현재의 시간을 표시한다. 
    time -y , -mo , -d , -h , -m , -s 
  - df 현재 시스템의 파일시스템 상태를 보여준다.
  - id : mode bus id를 보여준다. 변경시에는 뒤에 원하는 :ID를 적는다.
  - offset -i -v num value 
    임피던스 옵셋을 변경하고자 하면 
    offset -i 10 100을 적어 넣으면 1/100이 반영되며, -값은 "-100" 과 같이 적는다.
  

### Debugging Info.
- RTC Battery +, - Terminal SWAP
- USB-C type VCC Jumper to +5V and P5V
- RS통신 칩을 전이중 방식으로 사용되었기 때문에 칩을 교체 하였다. 
    - 회로도의 수정이 필요하다.

### Hardware Design

  - 내부의 LCD와 셀의 온도및 릴레이를 위해 사용한다.   
    - Serial1.begin(115200,SERIAL_8N1,26,22);
  - 외부 485통신에 사용한다.   
    - Serial2.begin(115200,SERIAL_8N1,18,19);
  - LCD와 셀의 통신을 선택하게 위형 2개의 GPIO를 사용한다.
    - IO 23 UAD1 
    - IO 2  UAD2 
  - **문제가 발생함.**  온도가 자꾸 올라간다. 

### LCD 모드버스 프로그램 시작.
    1. PC와의 에뮬레이션으로 시작한다. 
    2. PC쪽 프로그램은 Modscan을 사용하여 프로그램한다. 
    3. Address는 나중에 사용되는 External RS485와 같은 번지를 사용한다.
    4. 셀과의 통신이 항상 우선시 되므로 LCD에서는  매초 마다 폴링을 하여 요청이 있는지를 확인한다. 
    5. Device는 셀과의 통신이 완료 되면 LCD수신모드로 들어간다.(LCD에게는 Agent 이기 때문이다.)

### MODBUS FC04
1. FC04펑션의 추가 및 수정.    
  - 한번에 모든데이타를 전송한다. 최대 256개의 데이타   
  - 0x00~0xFF 
    - 0~39 : Cell Voltage
    - 40~79 : temperature + 40
    - 80~119 : impedance 
    - 120~255 : Etc 

1. FC03펑션의 추가 및 수정.    
  - 전압과 내부저항의 보정값루틴을 사용한다.
1. FC06을 사용하여 보정값을 메모리에 저장한다.


### Modbus Address function 3 Holding Register
  - read 0...1 (register 40001 to 40002)
    * send    01 03 00 00 00 02 C4 0B
    * receive 01 03 04 00 06 00 05 DA 31
### Modbus Address function 4 Input Register
  - read 0...1 (register 30001 to 30002)
    * send    01 04 00 00 00 02 71 CB
    * receive 01 04 04 00 06 00 05 DB 86 


### Modbus Address function 3,4
#### Fcode 04
    - 0~39: CellVolage * 100 
    - 40~79: temperature + 40
    - 80~119: impedance * 100 
#### Fcode 04
    - 0~39:  전압보정값
      전압 보정은 옵셋값으로 주며 0 번지에만 준다. 
      1옵셋당 약 9mv에 해당한다.
    - 40~79: not use 
    - 80~119: 임피던스보정값 

    - 120: year
    - 121: Month 
    - 122: day 
    - 123: Hour
    - 124: Minute 
    - 125: Second
    - 126: ModbusID
    - 127: installed_cells 
    - 128: AlarmTemperature 
    - 129: alarmHighCellVoltage 
    - 130: alarmLowCellVoltage 
    - 131: AlarmAmpere 
    #### function 6
    - 40 : year
    - 41 : Month 
    - 42 : day 
    - 43 : Hour
    - 44 : Minute 
    - 45 : Second