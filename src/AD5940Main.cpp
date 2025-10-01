/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Used to control specific application and process data.
 -----------------------------------------------------------------------------
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
/** 
 * @addtogroup AD5940_System_Examples
 * @{
 *  @defgroup Battery_Example
 *  @{
  */
#include <Arduino.h>
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "BATImpedance.h"
#include "ad5940.h"
#include <esp_task_wdt.h>
#include "SimpleCLI.h"
#include "mainClass.hpp"
#include "mainGrobal.h"
#include "batDeviceInterface.h"
#define MAX_LOOP_COUNT 60
#define APPBUFF_SIZE 512

static Print *outputStream;
uint32_t AppBuff[APPBUFF_SIZE];
char TAG[] = "AD5940";

extern _cell_value cellvalue[MAX_INSTALLED_CELLS];
/* It's your choice here how to do with the data. Here is just an example to print them to UART */
extern int measuredImpedance_1[MAX_INSTALLED_CELLS];
extern int measuredImpedance_2[MAX_INSTALLED_CELLS];
extern int measuredVoltage_1[MAX_INSTALLED_CELLS];
extern int measuredVoltage_2[MAX_INSTALLED_CELLS];
extern SimpleCLI simpleCli;
fImpCar_Type pImpResult[MAX_LOOP_COUNT +1];

SelectCell selectCell;
BatDeviceInterface batDevice;
static uint8_t selecectedCellNumber =0;
void AD5940_ShutDown();
bool AD5940_Calibration_ForLoop();
void addResult(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type Average;
  fImpCar_Type *pImp = (fImpCar_Type *)pData;
  // if (DataCount == 10)
  // {
  //   Average.Real = pImp->Real;
  //   Average.Image = pImp->Image;
  // }
  pImpResult[DataCount].Real = pImp->Real;
  pImpResult[DataCount].Image = pImp->Image;
  if (DataCount == (MAX_LOOP_COUNT - 1))
  {
    // MAX_LOOP_COUNT가 30이라면 20부터 시작해서 29까지 이나까.. 10개의 평균이다.
    Average.Real = pImp->Real;
    Average.Image = pImp->Image;
    for (int16_t i = MAX_LOOP_COUNT - 5; i < MAX_LOOP_COUNT; i++)
    {
      Average.Real += pImpResult[i].Real;
      Average.Real /= 2.0;
      Average.Image += pImpResult[i].Image;
      Average.Image /= 2.0;
    }
    ESP_LOGI("AVERAGE", "Average(real, image) = , %3.3f ,%3.3f ,%3.3f mOhm \n", Average.Real, Average.Image, AD5940_ComplexMag(&Average));
    outputStream->printf("\nAverage(real, image) = , %3.3f ,%3.3f ,%3.3f mOhm \n", Average.Real, Average.Image, AD5940_ComplexMag(&Average));
    // 보정값을 적용하여 주자
    float readImpdance ;
    readImpdance =  AD5940_ComplexMag(&Average);
    ESP_LOGI("AVERAGE", "Average(real, image) cellvalue[selecectedCellNumber ].impendance  %3.3f mOhm \n", 
      readImpdance  );
    readImpdance  += systemDefaultValue.impendanceCompensation[selecectedCellNumber ] / 100.0;
    cellvalue[selecectedCellNumber ].impendance= readImpdance ;
    ESP_LOGI("AVERAGE", "Average(real, image) cellvalue[selecectedCellNumber ].impendance  %3.3f mOhm \n", 
      readImpdance  );
    //위의 값은 다 버리고 다시 적용하자...이것은 임시로 적용한다.
    if(systemDefaultValue.runMode ==4)
    { // cheating mode
      float compensation = 0.0f;
      if (systemDefaultValue.modbusId == 1)
      {
        // 측정된 전압값을 반영 한다
        cellvalue[selecectedCellNumber].impendance =
            measuredImpedance_1[selecectedCellNumber]/100.0f;
        // 읽은 전압 값이 0.6V미만이면 임피던스는 0으로 놓는다.
        if (cellvalue[selecectedCellNumber].voltage < 0.6)
        {
          cellvalue[selecectedCellNumber].impendance = 0.0f;
        }
        float vGap = 10.0f * (measuredVoltage_1[selecectedCellNumber] / 100.0f - cellvalue[selecectedCellNumber].voltage) / float(measuredVoltage_1[selecectedCellNumber] / 100.0f); // 전압 변화량
        // 전압변화량이 +로 증가하면, 즉 기준값보다 읽은 값이 작다면 내부저항을 높여 준다.
        // 반대의 경우는 낮추어 준다
        // 전압변화량은 0~10까지 움직이므로 그 값을 그대로 합산한다.
        // 13.5V->12.5로 변했다면 0.74가 합산되어 진다.
        cellvalue[selecectedCellNumber].impendance += vGap;
      }
      else
      {
        cellvalue[selecectedCellNumber].impendance =
            measuredImpedance_2[selecectedCellNumber]/100.0f;
        // 읽은 전압 값이 4V미만이면 임피던스는 0으로 놓는다.
        if (cellvalue[selecectedCellNumber].voltage < 4)
          cellvalue[selecectedCellNumber].impendance = 0.0f;
        float vGap = 10.0f * (measuredVoltage_2[selecectedCellNumber] / 100.0f - cellvalue[selecectedCellNumber].voltage) / float(measuredVoltage_2[selecectedCellNumber] / 100.0f); // 전압 변화량
        cellvalue[selecectedCellNumber].impendance += vGap;
      }
    }
  }
}

int32_t BATShowResultBLE(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
	float freq;
	AppBATCtrl(BATCTRL_GETFREQ, &freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    outputStream->printf("Freq: %6.3f (real, image) = ,%6.3f , %6.3f ,%6.3f mOhm \n",freq, pImp[i].Real,pImp[i].Image,AD5940_ComplexMag(&pImp[i]));
  }
  return 0;

}
int32_t BATShowResult(char *tag, uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
	float freq;
	AppBATCtrl(BATCTRL_GETFREQ, &freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("\n%s Freq: %f (real, image) = %6.3f , %6.3f ,%6.3f mOhm \n",tag,freq, pImp[i].Real,pImp[i].Image,AD5940_ComplexMag(&pImp[i]));
    outputStream->printf("\nFreq: %f (real, image) = ,%6.3f , %6.3f ,%6.3f mOhm \n",freq, pImp[i].Real,pImp[i].Image,AD5940_ComplexMag(&pImp[i])); }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  /* Use hardware reset */
  ESP_LOGI(TAG,"AD5940_HWReset()");
  AD5940_HWReset();
  /* Platform configuration */
  ESP_LOGI(TAG,"AD5940_Initialize()");
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC; //on battery board, there is a 32MHz crystal.
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  ESP_LOGI(TAG,"AD5940_CLKCfg()");
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;                                /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  ESP_LOGI(TAG,"AD5940_FIFOCfg()");
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  ESP_LOGI(TAG,"AD5940_FIFOCfg()");
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  ESP_LOGI(TAG,"Step3. Interrupt controller ");
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP2_SYNC;
  gpio_cfg.InputEnSet = AGPIO_Pin0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin2 | AGPIO_Pin1;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  //AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* Allow AFE to enter sleep mode. */
  ESP_LOGI(TAG,"AD5940_SleepKeyCtrlS() ");
  delayMicroseconds(1000);
  return 0;
}

extern AppBATCfg_Type AppBATCfg ; 
void AD5940BATStructInit(void)
{
  AppBATCfg_Type *pBATCfg;
  AppBATGetCfg(&pBATCfg);
  pBATCfg->SeqStartAddr = 0;
  pBATCfg->MaxSeqLen = 512;
  pBATCfg->RcalVal = 56.0;  							/* Value of RCAL on EVAL-AD5941BATZ board is 50mOhm */
  pBATCfg->ACVoltPP = systemDefaultValue.ACVoltPP;							/* Pk-pk amplitude is 300mV */
  pBATCfg->DCVolt = systemDefaultValue.DCVolt;							/* Offset voltage of 1.2V*/
  pBATCfg->DftNum = DFTNUM_8192;
  
  pBATCfg->FifoThresh = 2;      					/* 2 results in FIFO, real and imaginary part. */
	
	pBATCfg->SinFreq = systemDefaultValue.SinFreq;									/* Sin wave frequency. THis value has no effect if sweep is enabled */
	
	pBATCfg->SweepCfg.SweepEn = bFALSE;			/* Set to bTRUE to enable sweep function */
	pBATCfg->SweepCfg.SweepStart = 900.0f;		/* Start sweep at 1Hz  */
	pBATCfg->SweepCfg.SweepStop = 1000.0f;	/* Finish sweep at 1000Hz */
	pBATCfg->SweepCfg.SweepPoints = 20;			/* 100 frequencies in the sweep */
	pBATCfg->SweepCfg.SweepLog = bTRUE;			/* Set to bTRUE to use LOG scale. Set bFALSE to use linear scale */
	
}
void AD5940_ShutDown(){
  AppBATCtrl(BATCTRL_SHUTDOWN,0);
}
extern TaskHandle_t *h_pxAD5940Task;
void AD5940_Main_reinit(){
  // 진행하고 있는 것을 멈추고 다시 시작을 해야 겠군.
  ESP_LOGI(TAG, "AD5940_Main_reinit: 주파수 변경 후 재초기화");
  
  AppBATCtrl(BATCTRL_STOPNOW, 0);
  // 설정 구조체 업데이트
  AppBATCfg_Type *pBATCfg;
  AppBATGetCfg(&pBATCfg);
  pBATCfg->ACVoltPP = systemDefaultValue.ACVoltPP;
  pBATCfg->DCVolt = systemDefaultValue.DCVolt;
  pBATCfg->DftNum = DFTNUM_8192;
  pBATCfg->FifoThresh = 2;
  pBATCfg->SinFreq = systemDefaultValue.SinFreq;  // 새로운 주파수 설정
  pBATCfg->bParaChanged = bTRUE;
  ESP_LOGI(TAG, "새로운 주파수 설정: %f Hz", systemDefaultValue.SinFreq);
  AppBATInit(AppBuff, APPBUFF_SIZE);
  
  // // AD5940을 완전히 재초기화
  // AD5940PlatformCfg();
  // AD5940BATStructInit();
  
  // // BAT 애플리케이션 재초기화
  // AD5940Err error = AppBATInit(AppBuff, APPBUFF_SIZE);
  // ESP_LOGW(TAG, "AppBATInit 재초기화 결과: %d %s", error, error == AD5940ERR_OK ? "성공" : "실패");
  
  // // 인터럽트 설정
  // AD5940_ClrMCUIntFlag();
  // AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
  
  ESP_LOGI(TAG, "AD5940 재초기화 완료");
}
void AD5940_Main_init()
{
  uint16_t temp;
  uint16_t iCount = 0;
  uint32_t startTime;
  ESP_LOGI(TAG, "AD5940_Main_init\n");
  AD5940PlatformCfg();
  AD5940BATStructInit();             /* Configure your parameters in this function */

  ESP_LOGW(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
  AD5940Err error = AppBATInit(AppBuff, APPBUFF_SIZE); /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  ESP_LOGW(TAG, "AppBATInit %d %s ",error ,error == AD5940ERR_OK ?"성공":"실패");

  // iCount = AD5940_WakeUp(50);
  // ESP_LOGI(TAG, "AD5940_Wakeup count is %d ",iCount);
  // vTaskDelay(50);
  // ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
}

/* Return RcalVolt magnitude 
* 
*/

float AD5940_calibration_read(float real , float image)
{
  AppBATCfg_Type BATCfg;
  BATCfg.RcalVolt.Real = real;
  BATCfg.RcalVolt.Image = image;
  return AD5940_ComplexMag(&BATCfg.RcalVolt);
}

#define CALIBRATION_LOOP_COUNT 10
float AD5940_calibration(float *real , float *image)
{
  simpleCli.outputStream->printf("Now on calibration(...");
  AD5940_Calibration_ForLoop();
  *real = AppBATCfg.RcalVolt.Real;
  *image = AppBATCfg.RcalVolt.Image;
  return AD5940_ComplexMag(&AppBATCfg.RcalVolt);
}
bool AD5940_Calibration_ForLoop(){
  bool retValue = false;
  uint16_t loopCount;
  AppBATCfg_Type beforRcalVolt;
  beforRcalVolt = AppBATCfg;
  double compareValue = 0.0f;
  AD5940PlatformCfg();
  AD5940BATStructInit();             /* Configure your parameters in this function */
  AppBATInit(AppBuff, APPBUFF_SIZE); /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  for (loopCount = 0; loopCount < systemDefaultValue.RcalLoopCount; loopCount++)
  {
    ESP_LOGW(TAG, "Reading Impedance(%d)", loopCount);
    beforRcalVolt.RcalVolt.Real = AppBATCfg.RcalVolt.Real;
    beforRcalVolt.RcalVolt.Image = AppBATCfg.RcalVolt.Image;
    AppBATCtrl(BATCTRL_MRCAL, 0); /* Measur RCAL each point in sweep */
    compareValue = abs(AD5940_ComplexMag(&beforRcalVolt.RcalVolt) - AD5940_ComplexMag(&AppBATCfg.RcalVolt)) / AD5940_ComplexMag(&AppBATCfg.RcalVolt);
    if (AD5940_ComplexMag(&AppBATCfg.RcalVolt) != 0.0f)
    {
      ESP_LOGI(TAG, "Real : %.3f, Image : %.3f, Mag : %.3f(%.3f) Compare:(%.4f)",
               AppBATCfg.RcalVolt.Real,
               AppBATCfg.RcalVolt.Image,
               AD5940_ComplexMag(&AppBATCfg.RcalVolt),
               AD5940_ComplexMag(&beforRcalVolt.RcalVolt), compareValue);
      simpleCli.outputStream->printf("Real : %.3f, Image : %.3f, Mag : %.3f(%.3f) Compare:(%.4f)",
               AppBATCfg.RcalVolt.Real,
               AppBATCfg.RcalVolt.Image,
               AD5940_ComplexMag(&AppBATCfg.RcalVolt),
               AD5940_ComplexMag(&beforRcalVolt.RcalVolt), compareValue);
      if (compareValue < 0.0001)
      {
        ESP_LOGI(TAG, "Finishe calibration");
        retValue = true;
        break;
      }
    }
    else
    {
      ESP_LOGI(TAG, "Not ready");
      simpleCli.outputStream->printf("Not ready");
    }
    simpleCli.outputStream->printf("\nOn Calibration(%d)", loopCount);
    delay(1000);
  }
  return retValue;
}
uint8_t isAD5940ReInit = 0;
void AD5940_Main_Loop()
{
  uint32_t temp;
  ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
  vTaskDelay(100);
  
  AppBATCtrl(BATCTRL_START, 0);
  static long elaspTime = 0;
  simpleCli.outputStream->printf("\nP1 : %d, P2 : %d, P3 : %d, P4 : %d, P5 : %d portNumber : %d", digitalRead(PORT1), digitalRead(PORT2), digitalRead(PORT3), digitalRead(PORT4), digitalRead(PORT5) 
      ,selectCell.getCurrentPort());
  ESP_LOGI(TAG, "\nP1:%d,P2:%d,P3:%d,P4:%d,P5:%d, portNumber : %d", 
      digitalRead(PORT1), digitalRead(PORT2), digitalRead(PORT3), digitalRead(PORT4), digitalRead(PORT5), 
      selectCell.getCurrentPort());
  
  fImpCar_Type pImpbuff[systemDefaultValue.RcalLoopCount/2];
  int successReadCount = 0;
  
  for (uint16_t loopCount = 0; loopCount < systemDefaultValue.RcalLoopCount; loopCount++)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    float batVoltage = 0.0;
    batVoltage = batDevice.readBatAdcValue(selecectedCellNumber, 600);
    printf("\nloopCount:%d, cell:%d, Bat Voltage : %f", loopCount, selecectedCellNumber, batVoltage);
    if (batVoltage > 18.0)
      batVoltage = 0.0;
    cellvalue[selecectedCellNumber - 1].voltage = batVoltage; // 구조체에 값을 적어 넣는다
    esp_task_wdt_reset();
    
    if (AD5940_GetMCUIntFlag())
    {
      AD5940_AGPIOToggle(AGPIO_Pin1);
      AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
      //AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppBATISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      //BATShowResult("NORMAL:",AppBuff, temp); /* Print measurement results over UART */
      
      // AppBuff에서 임피던스 값을 직접 읽어옴
      fImpCar_Type *pImp = (fImpCar_Type*)AppBuff;
      
      // 유효한 값이고 충분한 루프가 지났을 때만 처리
      BATShowResult("NORMAL:",AppBuff, temp); /* Print measurement results over UART */
      if(AD5940_ComplexMag(pImp) > 0.0001f && loopCount > (int)systemDefaultValue.RcalLoopCount*2/3)
      {
        // 유효한 측정값을 버퍼에 저장
        pImpbuff[successReadCount] = *pImp;
        successReadCount++;
        
        // 평균 계산
        fImpCar_Type pImpAvg = {0.0f, 0.0f};
        for(int i = 0; i < successReadCount; i++){
          pImpAvg.Real += pImpbuff[i].Real;
          pImpAvg.Image += pImpbuff[i].Image;
        }
        pImpAvg.Real /= successReadCount;
        pImpAvg.Image /= successReadCount;
        BATShowResult("APR:",AppBuff, temp); /* Print measurement results over UART */
        BATShowResult("AVG:",(uint32_t*)&pImpAvg, temp); /* Print measurement results over UART */
         
        // 평균값으로 임피던스 저장
        cellvalue[selecectedCellNumber - 1].impendance = AD5940_ComplexMag(&pImpAvg);
      }
      if(AD5940_ComplexMag(pImp) > 0.0001f){
        delay(100);
      }
      else{
        delay(1000);
      }
      printf("------------------------------->Loop:%d cell:%d\n",loopCount,selecectedCellNumber);
      if (isAD5940ReInit == 1)
      {
        isAD5940ReInit = 0;
        AD5940_Main_reinit();
        loopCount = 0;
        printf("AD5940_Main_reinit\n");
      }
      AD5940_SEQMmrTrig(SEQID_0); /* 정상 동작 확인 완료 Trigger next measurement ussing MMR write*/
    }
  }
};
void setSelectCell(uint8_t cellNumber)
{
  selecectedCellNumber = cellNumber;
}
void AD5940_Main(void *parameters)
{
  AD5940_MCUResourceInit(0);
  AD5940_Main_init();
  AppBATInit(AppBuff, APPBUFF_SIZE); /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  AppBATCfg.RcalVolt.Real = systemDefaultValue.real_Cal;
  AppBATCfg.RcalVolt.Image = systemDefaultValue.image_Cal;
  if (parameters != nullptr)
    outputStream = static_cast<Print *>(parameters);
  if (outputStream == nullptr)
  {
    outputStream = &Serial;
  }
  bool isCalibrated = false;
  selecectedCellNumber = 1;
  for (;; selecectedCellNumber++)
  {
    if (systemDefaultValue.runMode != 0)
    {
      if (selecectedCellNumber > systemDefaultValue.installed_cells)
        selecectedCellNumber = 1;
      selectCell.select(selecectedCellNumber);
      // selecectedCellNumber = selecectedCellNumber == 1 ? 2 : 1;
      AppBATCfg.RcalVolt.Real = systemDefaultValue.real_Cal;
      AppBATCfg.RcalVolt.Image = systemDefaultValue.image_Cal;
      printf("RcalVolt Real : %.3f, Image : %.3f, Mag : %.3f\n", AppBATCfg.RcalVolt.Real, AppBATCfg.RcalVolt.Image,AD5940_ComplexMag(&AppBATCfg.RcalVolt));
      outputStream->printf("RcalVolt Real : %.3f, Image : %.3f, Mag : %.3f\n", AppBATCfg.RcalVolt.Real, AppBATCfg.RcalVolt.Image,AD5940_ComplexMag(&AppBATCfg.RcalVolt));  
      // if (!isCalibrated)
      // {
      //   AppBATCfg.RcalVolt.Real = systemDefaultValue.real_Cal;
      //   AppBATCfg.RcalVolt.Image = systemDefaultValue.image_Cal;
      //   isCalibrated = AD5940_Calibration_ForLoop();
      // }
      AD5940_Main_Loop();
    }
    else
    {
        printf("RcalVolt Real : %.3f, Image : %.3f, Mag : %.3f\n", AppBATCfg.RcalVolt.Real, AppBATCfg.RcalVolt.Image,AD5940_ComplexMag(&AppBATCfg.RcalVolt));
        outputStream->printf("\nRcalVolt Real : %.3f, Image : %.3f, Mag : %.3f\n", AppBATCfg.RcalVolt.Real, AppBATCfg.RcalVolt.Image,AD5940_ComplexMag(&AppBATCfg.RcalVolt));  
        printf("AD5940 Running Mode is %d\n",systemDefaultValue.runMode);
        outputStream->printf("\nAD5940 Running Mode is %d\n",systemDefaultValue.runMode);
        vTaskDelay(1000);
    }
    vTaskDelay(100);
  }
}

/**
 * @}
 * @}
 * */
