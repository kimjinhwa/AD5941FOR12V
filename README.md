# TIA Resistance Measurement System.
## �ý��ۼ���
  * �ʱ� ������ ON�Ҷ��� �ݵ�� ������� ������ �и��� ��    
    ��� ����ġ�� OFF���� ���� �� �Ŀ��� �����Ѵ�.
  * runmode  
    - 0 : Nothing
    - 1 : Read Voltage 
    - 2 : Not Defined 
    - 3 : Read Impedance 
    - 4 : Impedance Cheating Mode. 
  * bat <num>
    - ��ġ�� ������ ���� �����Ѵ�. 
    - �ڵ����� �ý����� �� ���õȴ�.
  * id <num>
    - modbus id �� �����Ѵ�. 
  * offset [i][ia][v][va][vv] <num> <value>
     - -ia : ��� ���� ���Ͽ� ���������Ѵ�.
     - -iv : num�� 0���� �ָ� ��� ���� ���Ͽ� ���������Ѵ�.
     - offset 0 0 ������ ������ ���� �����ش�.
       �뷫������ 1 ���ܿ� 15mv�� ����.   
       offset 0 "-36"  �� ���� �Է��Ѵ�
  * cal : calibration 
     - ������ ������ ���װ����� ���ذ��� �д´�.
     - cal save : ���� �����ϰ� EEPROM�� �����Ѵ�. �ѹ� �ߴٸ� ������ ���� Ȱ���Ͽ� ����Ѵ�.
  * time : �ð� ����
    - time -y 2024 -mo 9 -d 12 -h 09 -m 47 -s 00 
  * reboot : �ý����� �� �����Ѵ�.
  * p15relay : ���κ��忡�� �������忡 �����ϴ� ��ȣ 15V�� ��� �Ѵ�. 
    - p15relay 00 :  ���� ������ �� ���� ��� P15�� ����Ѵ�.   
     �̰��� ��������� ���� ����̴�. �� �̻��¿��� ������ ������ ���������� ���� ����� ���õȴ�. 
  - p15relay 01 : 
  - p15relay 10 : 
  - p15relay 11 : 
* mod/uleid : ����� ID�� ��Ʈ�� ���� �����ϰ��� �� �� ����� �� �ִ�. 
  - ��� ���̺��� ��ü���� �и��Ѵ�.
  - runmode 0 ���� ���� �ý����� �� �����Ѵ�.
  - p15relay 00 �� �ؼ� ���� ���ö��ο� ������ �����ϰ�    
    ���̽��� ���� ���۸� �� ��  ���̺� �ϳ��� ���θ� �����Ѵ�. 
  
  - mod <number> �� ���̵� �����Ѵ�.
  - mod �� �Է��Ͽ� ������ id�� Ȯ���Ѵ�.  ## ���� ���� 

  *  checkVoltageoff(): [Voltage] Bat OFF Check Voltage is : 18.994 
## BOARD DEBUGGING
  * WIFI ���׳� �κ��� �Ǵ�.
  * R1  100K�� 200K�� .
  * P15V��¿� Toggle Switch�� �޾� �ش�.
  * ADG656_SEL IO27 �� ���۸� ������� �ʰ� GPIO2_EXT�� Ȯ���Ѵ� ( J2 ����)
  * SENSE_P �� SENSE_N�� �ٲ��ش�.(ȸ�ε�����.. ����� ������ �̷��� ���ִ°� ����)
  *  adc_filter.Sinc2NotchEnable = bTRUE; �̰��� Ȱ��ȭ �Ǿ� ���� �ʴ�.    
     �ٽ� Ȯ���� ����. 
## BLUETOOTH COMMAND
### offset #
 #### offset    [-ia cellno value ] [-va  cellno value ]    [-i  cellno value ] 
 * [-ia cellno value ] ���Ǵ����� ���� �ɼ��� �����. 
   * cellno : ����ȣ 1�����ͽ����Ѵ�.   
     0 �� �־����� ��缿�� ���Ͽ� �����Ѵ�.   
   * value    
         systemDefaultValue.impendanceCompensation[i]�� 
         ���� �־��� ���� += �Ѵ�.   
         ```
          cellvalue[i].impendance = cellvalue[i].impendance +   systemDefaultValue.impendanceCompensation[i] / 100.f;   
          ```
      ���� Value�� 1/100�� �����ϱ� ������ 100�� ���� 
      ������ �ش�.   
 * [-va  cellno value ]  ���п� ���� �ɼ��� �����. 
      * cellno : ����ȣ 1�����ͽ����Ѵ�.   
                  0 �� �־����� ��缿�� ���Ͽ� �����Ѵ�.   
      * value    
         systemDefaultValue.voltageCompensation[i]�� 
         ���� �־��� ���� += �Ѵ�.     
        ``` 
         systemDefaultValue.voltageCompensation[i] += value 
        ```
      ���� Value�� 1/100�� �����ϱ� ������ 100�� ���� 
      ������ �ش�.   
 * [-i cellno value ] ia�� �����ϳ� +=�� �ƴ� =�� ����Ѵ�.
      ```
      cellvalue[i].impendance = cellvalue[i].impendance + systemDefaultValue.impendanceCompensation[i] / 100.f;
      ```
 * [-v cellno value ] va�� �����ϳ� +=�� �ƴ� =�� ����Ѵ�.
      ```
        systemDefaultValue.voltageCompensation[i] = value; // cellvalue[0].voltage - cellvalue[i].voltage;
        cellvalue[number - 1].voltage= cellvalue[number - 1].voltage+ systemDefaultValue.voltageCompensation[number - 1];
      ```

 * [-vv cellno value ] �Է��� ������ �ɼ��� ���߾� �ִ� �ɼ��̴�.     
 ����� ���ϱ� �� �ֱ� ���� ����̴�.   
 ù��°�� ����ȣ�� �ι�°�� ���ϴ� ���а��� ����Ѵ�
 ```
    float gapVoltage = fvalue - cellvalue[number-1].voltage ;
    int voloffset = (int)(gapVoltage /0.005);
    systemDefaultValue.voltageCompensation[number - 1] = voloffset;
    //voltageCompensation���� �Ʒ��� ���� adcReading Offset�� ���ȴ�.
    systemDefaultValue.voltageCompensation[number - 1] = voloffset;
  uint32_t voltage = esp_adc_cal_raw_to_voltage((uint32_t)batVoltageAdcValue , &adc_chars);
 ``` 

# relay -s [Cell no] -off 
  - relay -s [Cell no] -off 
    - off : check if all cell is offed 
    - Cell no : 0 All Off, 
      number : Set Relay On the number and +1 relay on 
  - cal/ibration ���� ���Ǵ����� ���Ѵ�. 
    cal save : ���� �а� �����Ѵ�.
    cal save Ķ���극�̼� �� ���� EEPROM�� �����Ѵ�
  - mode  �ڵ����� ���Ǵ����� ���� ������ �ƴ����� �����Ѵ�. 
          Calibration�� �����ϱ� ���ؼ��� �̰��� �������� ���� �Ǿ� �־�� �Ѵ�.
    - mode 0 :����
    - mode 1 : ���и� �ڵ�
    - mode 3 : ���а� ���Ǵ��� �ڵ� 
  - bat/number [40]: ��ġ�Ǿ��ִ� ���͸����ڸ� ����Ѵ�
  - temp/erature : ��ü�� �µ��� �д� ��ƾ�̴�.
  - time : ������ �ð��� ǥ���Ѵ�. 
    time -y , -mo , -d , -h , -m , -s 
  - df ���� �ý����� ���Ͻý��� ���¸� �����ش�.
  - id : mode bus id�� �����ش�. ����ÿ��� �ڿ� ���ϴ� :ID�� ���´�.
  - offset -i -v num value 
    ���Ǵ��� �ɼ��� �����ϰ��� �ϸ� 
    offset -i 10 100�� ���� ������ 1/100�� �ݿ��Ǹ�, -���� "-100" �� ���� ���´�.
  

### Debugging Info.
- RTC Battery +, - Terminal SWAP
- USB-C type VCC Jumper to +5V and P5V
- RS��� Ĩ�� ������ ������� ���Ǿ��� ������ Ĩ�� ��ü �Ͽ���. 
    - ȸ�ε��� ������ �ʿ��ϴ�.

### Hardware Design

  - ������ LCD�� ���� �µ��� �����̸� ���� ����Ѵ�.   
    - Serial1.begin(115200,SERIAL_8N1,26,22);
  - �ܺ� 485��ſ� ����Ѵ�.   
    - Serial2.begin(115200,SERIAL_8N1,18,19);
  - LCD�� ���� ����� �����ϰ� ���� 2���� GPIO�� ����Ѵ�.
    - IO 23 UAD1 
    - IO 2  UAD2 
  - **������ �߻���.**  �µ��� �ڲ� �ö󰣴�. 

### LCD ������ ���α׷� ����.
    1. PC���� ���ķ��̼����� �����Ѵ�. 
    2. PC�� ���α׷��� Modscan�� ����Ͽ� ���α׷��Ѵ�. 
    3. Address�� ���߿� ���Ǵ� External RS485�� ���� ������ ����Ѵ�.
    4. ������ ����� �׻� �켱�� �ǹǷ� LCD������  ���� ���� ������ �Ͽ� ��û�� �ִ����� Ȯ���Ѵ�. 
    5. Device�� ������ ����� �Ϸ� �Ǹ� LCD���Ÿ��� ����.(LCD���Դ� Agent �̱� �����̴�.)

### MODBUS FC04
1. FC04����� �߰� �� ����.    
  - �ѹ��� ��絥��Ÿ�� �����Ѵ�. �ִ� 256���� ����Ÿ   
  - 0x00~0xFF 
    - 0~39 : Cell Voltage
    - 40~79 : temperature + 40
    - 80~119 : impedance 
    - 120~255 : Etc 

1. FC03����� �߰� �� ����.    
  - ���а� ���������� ��������ƾ�� ����Ѵ�.
1. FC06�� ����Ͽ� �������� �޸𸮿� �����Ѵ�.


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
    - 0~39:  ���к�����
      ���� ������ �ɼ°����� �ָ� 0 �������� �ش�. 
      1�ɼ´� �� 9mv�� �ش��Ѵ�.
    - 40~79: not use 
    - 80~119: ���Ǵ��������� 

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