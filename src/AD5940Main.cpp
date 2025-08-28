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

#define MAX_LOOP_COUNT 30
#define APPBUFF_SIZE 512

static Print *outputStream;
uint32_t AppBuff[APPBUFF_SIZE];
char TAG[] = "AD5940";

extern uint8_t selecectedCellNumber ;
extern _cell_value cellvalue[MAX_INSTALLED_CELLS];
/* It's your choice here how to do with the data. Here is just an example to print them to UART */
extern int measuredImpedance_1[20];
extern int measuredImpedance_2[20];
extern int measuredVoltage_1[20];
extern int measuredVoltage_2[20];
extern SimpleCLI simpleCli;
fImpCar_Type pImpResult[MAX_LOOP_COUNT +1];

void AD5940_ShutDown();
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
int32_t BATShowResult(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
	float freq;
	AppBATCtrl(BATCTRL_GETFREQ, &freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("Freq: %f (real, image) = %6.3f , %6.3f ,%6.3f mOhm \n",freq, pImp[i].Real,pImp[i].Image,AD5940_ComplexMag(&pImp[i]));
    outputStream->printf("Freq: %f (real, image) = ,%6.3f , %6.3f ,%6.3f mOhm \n",freq, pImp[i].Real,pImp[i].Image,AD5940_ComplexMag(&pImp[i]));
  }
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
  fifo_cfg.FIFOThresh = 4;//AppBATCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  ESP_LOGI(TAG,"AD5940_FIFOCfg()");
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  ESP_LOGI(TAG,"AD5940_FIFOCfg()");
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  ESP_LOGI(TAG,"Step3. Interrupt controller ");
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
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
  pBATCfg->ACVoltPP = 800.0f;							/* Pk-pk amplitude is 300mV */
  pBATCfg->DCVolt = 1100.0f;							/* Offset voltage of 1.2V*/
  pBATCfg->DftNum = DFTNUM_8192;
  
  pBATCfg->FifoThresh = 2;      					/* 2 results in FIFO, real and imaginary part. */
	
	pBATCfg->SinFreq = 5000/5;									/* Sin wave frequency. THis value has no effect if sweep is enabled */
	
	pBATCfg->SweepCfg.SweepEn = bFALSE;			/* Set to bTRUE to enable sweep function */
	pBATCfg->SweepCfg.SweepStart = 900.0f;		/* Start sweep at 1Hz  */
	pBATCfg->SweepCfg.SweepStop = 1000.0f;	/* Finish sweep at 1000Hz */
	pBATCfg->SweepCfg.SweepPoints = 20;			/* 100 frequencies in the sweep */
	pBATCfg->SweepCfg.SweepLog = bTRUE;			/* Set to bTRUE to use LOG scale. Set bFALSE to use linear scale */
	
}
void AD5940_ShutDown(){
  AppBATCtrl(BATCTRL_SHUTDOWN,0);
}
void AD5940_Main_init()
{
  uint16_t temp;
  uint16_t iCount = 0;
  fImpCar_Type beforRcalVolt;
  uint32_t startTime;
  ESP_LOGI(TAG, "AD5940_Main_init\n");
  AD5940PlatformCfg();
  AD5940BATStructInit();             /* Configure your parameters in this function */

  AD5940Err error = AppBATInit(AppBuff, APPBUFF_SIZE); /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  ESP_LOGI(TAG, "AppBATInit %d %s ",error ,error == AD5940ERR_OK ?"성공":"실패");


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
  AppBATCfg.RcalVolt.Real = real;
  AppBATCfg.RcalVolt.Image = image;
  return AD5940_ComplexMag(&AppBATCfg.RcalVolt);
}

#define CALIBRATION_LOOP_COUNT 10
float AD5940_calibration(float *real , float *image)
{
  uint16_t loopCount = CALIBRATION_LOOP_COUNT;
  AD5940PlatformCfg();
  AD5940BATStructInit();             /* Configure your parameters in this function */
  AppBATInit(AppBuff, APPBUFF_SIZE); /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  *real = 0.0f;
  *image = 0.0f;
  simpleCli.outputStream->printf("Now on calibration(...");
  while (loopCount--)
  {
    //simpleCli.outputStream->
    //simpleCli.outputStream->printf("Now on calibration(%d)...", loopCount);
    time_t startTime = millis();
    if (AD5940ERR_WAKEUP == AppBATCtrl(BATCTRL_MRCAL, 0))
    {
      simpleCli.outputStream->printf("\nWakeup Error..retry...");
    }; /* Measur RCAL each point in sweep */
    time_t endTime = millis();
    simpleCli.outputStream->printf("\r\n%d: R I Mag:%6.2f\t %6.2f\t %6.2f (%dmills)",
                           loopCount,
                           AppBATCfg.RcalVolt.Real,
                           AppBATCfg.RcalVolt.Image,
                           AD5940_ComplexMag(&AppBATCfg.RcalVolt), endTime - startTime);
    delay(100);
    if (loopCount < CALIBRATION_LOOP_COUNT)
    {
      *real += AppBATCfg.RcalVolt.Real;
      *image += AppBATCfg.RcalVolt.Image;
    }
  };
  *real /= CALIBRATION_LOOP_COUNT;
  *image /= CALIBRATION_LOOP_COUNT;
  AppBATCfg.RcalVolt.Real = *real;
  AppBATCfg.RcalVolt.Image = *image;
  AD5940_ShutDown();
  return AD5940_ComplexMag(&AppBATCfg.RcalVolt);
}
void AD5940_Main(void *parameters)
{
  uint32_t temp;
  if(parameters != nullptr)
  outputStream  = static_cast<Print *>(parameters);
  if(outputStream == nullptr)
  {
    outputStream = &Serial;
  }

  AD5940_MCUResourceInit(0);
  AD5940_Main_init();
  ESP_LOGI(TAG, "Chip Id : %d\n", AD5940_ReadReg(REG_AFECON_CHIPID));
  vTaskDelay(100);
  AD5940PlatformCfg();
  AD5940BATStructInit(); /* Configure your parameters in this function */
  AppBATInit(AppBuff, APPBUFF_SIZE);    /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */

  AppBATCfg.RcalVolt.Real = systemDefaultValue.real_Cal;
  AppBATCfg.RcalVolt.Image = systemDefaultValue.image_Cal; 

  uint16_t loopCount ;
  AppBATCtrl(BATCTRL_MRCAL, 0);     /* Measur RCAL each point in sweep */
  AppBATCtrl(BATCTRL_START, 0);
  AD5940_ClrMCUIntFlag(); /* Clear this flag */
  time_t startTime = millis();
  // ESP_LOGI(TAG, "Waiting for MC UInt Flag ...");
  // ESP_LOGI(TAG, "MC UInt Flag ...Elasped %d",millis()-startTime);
  startTime = millis();
	AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); // 이것이 동작 하는 것은 확인했다.
  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(loopCount < MAX_LOOP_COUNT)
      loopCount++;
    else
      loopCount = 0;
    esp_task_wdt_reset();
    if(AD5940_GetMCUIntFlag())
    {
      //ESP_LOGI(TAG, "Reading Impedance(%d)",MAX_LOOP_COUNT - loopCount);
      AD5940_AGPIOToggle(AGPIO_Pin1);
      AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      //AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
      AppBATISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      //AD5940_Delay10us(10000);
      delay(1000);
      //addResult(AppBuff, loopCount);
      BATShowResult(AppBuff, temp); /* Print measurement results over UART */
      printf("--------------------------------\n");
      AD5940_SEQMmrTrig(SEQID_0);   /* 정상 동작 확인 완료 Trigger next measurement ussing MMR write*/
      //ESP_LOGI(TAG, "Ended Impedance elasped %d",millis()-startTime);
    }
    delay(1);
  }
}

/**
 * @}
 * @}
 * */
