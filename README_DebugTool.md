# Modbus Debug Explaination
  * Send data from PC via RS232 modbus and receive at controller
  * Then Controller send data to ad5941 via SPI and receive data.
  * This received data was sent to PC
## Controll SUB SENSOR MODULE 
  * To controll sub module via PC modbus
### Address Definition
  * FC01 30001 : -> Relay 1 Read 
  * FC01 30002 : -> Relay 1 Read 

  * FC03 30001 : -> Module Address 0 Not use
  * FC03 30002 : -> Module Address 1 Change Modbus ID
  * FC03 30003 : -> Module Address 2 Change Comm Speed 

  * FC04 30001 : -> Module Address 0 Temperature 
  * FC04 30002 : -> Module Address 1 Modbus ID
  * FC04 30003 : -> Module Address 2 Comm Speed 

  * FC05 30001 : -> Relay 1 Write
  * FC05 30002 : -> Relay 1 Write
## Controll AD5941
### Using modbus
- For controll AD5941 via modbus 
- FC03 : Read Register at ad4941
  **Most High Address** REG_AFE_ADCFILTERCFG: 0x00004010   
  **Most Low Address**  REG_AFECRC_CTL: 0x00001000
- address start from 30001 or 40001

### Modbus Address function 3 Holding Register
  - read 0...1 (register 40001 to 40002)
    * send    01 03 00 00 00 02 C4 0B
    * receive 01 03 04 00 06 00 05 DA 31
### Modbus Address function 4 Input Register
  - read 0...1 (register 30001 to 30002)
    * send    01 04 00 00 00 02 71 CB
    * receive 01 04 04 00 06 00 05 DB 86 

### MODBUS FC06
  - FC06을 사용하여 보정값을 메모리에 저장한다.

## BLUETOOTH COMMAND
``` 
SimpleCLI::SimpleCLI(int commandQueueSize, int errorQueueSize,Print *outputStream ) : commandQueueSize(commandQueueSize), errorQueueSize(errorQueueSize) {
  this->inputStream = &Serial;
  Command cmd_config = addCommand("ls",ls_configCallback);
  cmd_config.setDescription(" File list \r\n ");
  cmd_config =  addSingleArgCmd("cat", cat_configCallback);
  cmd_config = addSingleArgCmd("rm", rm_configCallback);
  cmd_config = addSingleArgCmd("format", format_configCallback);
  cmd_config = addCommand("time", time_configCallback);
  cmd_config.addArgument("y/ear","");
  cmd_config.addArgument("mo/nth","");
  cmd_config.addArgument("d/ay","");
  cmd_config.addArgument("h/our","");
  cmd_config.addArgument("m/inute","");
  cmd_config.addArgument("s/econd","");
  cmd_config.setDescription(" Get Time or set \r\n time -y 2024 or time -M 11,..., Month is M , minute is m ");
  cmd_config = addCommand("df", df_configCallback);
  cmd_config = addSingleArgCmd("reboot", reboot_configCallback);

  cmd_config = addCommand("r/elay", relay_configCallback);
  cmd_config.addArgument("s/el","");
  cmd_config.addArgument("t/est","");
  cmd_config.addFlagArg("off");
  cmd_config.setDescription("relay on off controll \r\n relay -s/el [1] [-off]");
  cmd_config = addSingleArgCmd("mode", mode_configCallback);
  cmd_config = addSingleArgCmd("id", id_configCallback);
  cmd_config = addSingleArgCmd("cal/ibration", calibration_configCallback);
  cmd_config = addSingleArgCmd("bat/number", batnumber_configCallback);
  cmd_config = addCommand("imp/edance", impedance_configCallback);
  cmd_config = addSingleArgCmd("temp/erature", temperature_configCallback);
  cmd_config = addSingleArgCmd("mod/uledid", moduleid_configCallback);

  cmd_config = addCommand("writecellLog",writeCellLog_configCallback);
  cmd_config = addCommand("readcellLog",readCellLog_configCallback);
  cmd_config = addCommand("wrm",measuredvalue_configCallback);
  cmd_config.addPositionalArgument("num");
  cmd_config.addPositionalArgument("imp");
  cmd_config.addPositionalArgument("vol");
  cmd_config.setDescription("\n \
    Input measured impdance and voltage compansation value\n  \
    Usage : wrm cellno imp vol \n\
  ");

  cmd_config = addCommand("offset",offset_configCallback);
  cmd_config.addFlagArgument("i"); //impedance
  cmd_config.addFlagArgument("ia"); //impedance
  cmd_config.addFlagArgument("v"); //voltage
  cmd_config.addFlagArgument("va"); //voltage
  cmd_config.addFlagArgument("vv"); //voltage
  cmd_config.addPositionalArgument("num");
  cmd_config.addPositionalArgument("value");
  cmd_config.setDescription("\nFor impdance and voltage compansation value\n  \
    Usage: offset [-i (cellnumber) (value)] [-v (cellnumber) (value)]\n    \
                [-ia][-va]  \
           flag: -ia  0 [value] [ set all offset value set to samevalue to given impedance value]\n   \
           flag: -iv  [ for set to samevalue to First voltage value]\n     \
           For set IMP input, value is 100 times.\n, \
           For set Vol input, value is for use offset. So not multiples.\n, \
             ");
  simpleCli.setOnError(errorCallback);
  cmd_config= addCommand("help",help_Callback);
}
```
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
