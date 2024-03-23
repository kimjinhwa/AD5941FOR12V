## TIA Resistance Measurement System.
## BLUETOOTH COMMAND
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


### Modbus Address function 4(3000)

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