#include "myBlueTooth.h"
#include "SimpleCLI.h"
#include <WiFi.h>
#include <BluetoothSerial.h>
#include "filesystem.h"
#include <RtcDS1302.h>


extern BluetoothSerial SerialBT;
//extern LittleFileSystem lsFile;
// Print *outputStream;
// Stream *inputStream;


// void ls_configCallback(cmd *cmdPtr){
//   Command cmd(cmdPtr);
//   Argument arg = cmd.getArgument(0);
//   String argVal = arg.getValue();
//   lsFile.listDir("/spiffs/", NULL);
// }


// void time_configCallback(cmd *cmdPtr){
//   RtcDateTime now;
//   Command cmd(cmdPtr);
//   Argument arg ;
//   String strValue ;
//   // gettimeofday(&tmv,NULL);
//   // struct tm *timeinfo = gmtime(&tmv.tv_sec);
//   // now =  RtcDateTime (tmv.tv_sec); //
//   //struct timeval tmv;
//   now = getDs1302GetRtcTime();
//   outputStream->printf("\nnow is to %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
//   //tmv.tv_usec = now.TotalSeconds();

//   arg = cmd.getArgument("year");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     outputStream->printf("\nTime will Set to %d/%d/%d %d:%d:%d ",strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
//     now = RtcDateTime(strValue.toInt(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
//     setRtcNewTime(now);
//     now = getDs1302GetRtcTime();
//     outputStream->printf("\nNow New time is set to %d/%d/%d %d:%d:%d ",now.Year(),now.Month(),now.Day(),now.Hour(),now.Minute(),now.Second());
//     return ;
//   }
//   arg = cmd.getArgument("Month");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     now = RtcDateTime(now.Year(),strValue.toInt(),now.Day(),now.Hour(),now.Minute(),now.Second());
//     setRtcNewTime(now);
//     return ;
//   }
//   arg = cmd.getArgument("day");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     now = RtcDateTime(now.Year(),now.Month(),strValue.toInt(),now.Hour(),now.Minute(),now.Second());
//     setRtcNewTime(now);
//     return ;
//   }
//   arg = cmd.getArgument("hour");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     now = RtcDateTime(now.Year(),now.Month(),now.Day(),strValue.toInt(),now.Minute(),now.Second());
//     setRtcNewTime(now);
//     return ;
//   }
//   arg = cmd.getArgument("minute");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     now = RtcDateTime(now.Year(),now.Month(),now.Day(),now.Hour(),strValue.toInt(),now.Second());
//     setRtcNewTime(now);
//     return ;
//   }
//   arg = cmd.getArgument("second");
//   if(arg.isSet()){
//     strValue = arg.getValue();
//     return ;
//   }
//   getTime();
//   String argVal = arg.getValue();
// }

// void errorCallback(cmd_error *errorPtr)
// {
//   CommandError e(errorPtr);

//   outputStream->printf( (String("ERROR: ") + e.toString()).c_str());

//   if (e.hasCommand())
//   {
//     outputStream->printf(  (String("Did you mean? ") + e.getCommand().toString()).c_str());
//   }
//   else
//   {
//      outputStream->printf(  simpleCli.toString().c_str());
//   }
// }

// void help_Callback(cmd *cmdptr){

// }
// void df_configCallback(cmd *cmdPtr)
// {
//   Command cmd(cmdPtr);
//   lsFile.df();
// }
// void reboot_configCallback(cmd *cmdPtr)
// {
//   Command cmd(cmdPtr);
//   outputStream->println( "\r\nNow System Rebooting...\r\n");
//   esp_restart();
// }

// void format_configCallback(cmd *cmdPtr)
// {
//   outputStream->printf("\r\nWould you system formating(Y/n)...\r\n");
//   int c = 0;
//   while (1)
//   {
//     if (SerialBT.connected())
//       c = SerialBT.read();
//     else
//       c = Serial.read();

//     if (c == 'Y')
//     {
//       lsFile.littleFsInit(1);
//       outputStream->printf("\r\nSystem format completed\r\n");
//       return;
//     }
//     if (c == 'n')
//       return;
//   }
// }

// void rm_configCallback(cmd *cmdPtr)
// {
//   Command cmd(cmdPtr);
//   Argument arg = cmd.getArgument(0);
//   String argVal = arg.getValue();
//   outputStream->printf("\r\n");

//   if (argVal.length() == 0)
//     return;

//   lsFile.rm(argVal);
//   // if (!argVal.startsWith("*"))
//   // {
//   //   argVal = String("/spiffs/") + argVal;
//   // }
// };
// void cat_configCallback(cmd *cmdPtr)
// {
//   Command cmd(cmdPtr);
//   Argument arg = cmd.getArgument(0);
//   String argVal = arg.getValue();
//   outputStream->printf("\r\n");

//   if (argVal.length() == 0)
//     return;
//   argVal = String("/spiffs/") + argVal;

//   lsFile.cat(argVal);
// }
myBlueTooth::myBlueTooth(){

  //Command cmd_config ;//= simpleCli.addCommand("ls",ls_configCallback);
  //cmd_config =  simpleCli.addSingleArgCmd("cat", cat_configCallback);
  //cmd_config = simpleCli.addSingleArgCmd("rm", rm_configCallback);
  //cmd_config = simpleCli.addSingleArgCmd("mv", mv_configCallback);
  //cmd_config = simpleCli.addSingleArgCmd("format", format_configCallback);
  //cmd_config = simpleCli.addCommand("time", time_configCallback);
  // cmd_config.addArgument("y/ear","");
  // cmd_config.addArgument("M/onth","");
  // cmd_config.addArgument("d/ay","");
  // cmd_config.addArgument("h/our","");
  // cmd_config.addArgument("m/inute","");
  // cmd_config.addArgument("s/econd","");
//  cmd_config.setDescription(" Get Time or set \r\n time -y 2024 or time -M 11,..., Month is M , minute is m ");
//  cmd_config = simpleCli.addCommand("df", df_configCallback);
  //cmd_config = simpleCli.addSingleArgCmd("reboot", reboot_configCallback);
  //simpleCli.setOnError(errorCallback);
  //cmd_config= simpleCli.addCommand("help",help_Callback);
  //cmd_config.setDescription(" Get help!");
}

void myBlueTooth::readInputSerialBT()
{
  char readBuf[2];
  char readCount = 0;
  // while (true)
  {
    if (SerialBT.available())
    {
      if (SerialBT.readBytes(readBuf, 1))
      {
        readBuf[1] = 0x00;
        if (readBuf[0] == 8)
        {
          input.remove(input.length() - 1);
        }
        else
        {
          printf("%c", readBuf[0]);
          input += String(readBuf);
        }
      }
      if (readBuf[0] == '\n' || readBuf[0] == '\r')
      {
        simpleCli.parse(input);
        while (SerialBT.available()) SerialBT.readBytes(readBuf, 1);
        input = "";
        printf("\n# ");
        // break;
      }
    }
  }
}
// void btCallBack(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
//   if (event == ESP_SPP_OPEN_EVT                    ) {
//     Serial.printf("\nUser Connected");
//   }

// };
static unsigned long previousmills = 0;
static int everySecondInterval = 5000;
static unsigned long now;
void blueToothTask(void *parameter)
{
  myBlueTooth blueTooth;
  //outputStream = &Serial;
  simpleCli.inputStream= &Serial;
//  SerialBT.register_callback((esp_spp_cb_t *)btCallBack);
// typedef void (*esp_spp_cb_t)(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
// ESP_SPP_OPEN_EVT                    


  for (;;)
  {
    blueTooth.readInputSerialBT();
    now = millis();
    if ((now - previousmills > everySecondInterval))
    {
      previousmills = now;
      if (SerialBT.connected()){
        lsFile.setOutputStream(&SerialBT);
        simpleCli.outputStream = &SerialBT;
        simpleCli.inputStream= &SerialBT;
      }
      else{
        lsFile.setOutputStream(&Serial);
        simpleCli.outputStream = &Serial;
        simpleCli.inputStream= &Serial;
      }
    }
    delay(10);
  }
}
