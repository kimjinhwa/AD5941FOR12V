/*!
 *****************************************************************************
 @file:    ADICUP3029Port.c
 @author:  Neo Xu
 @brief:   The port for ADI's ADICUP3029 board.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <AD5940.h>
#include "mainGrobal.h"
#include "driver/gpio.h"


#define SYSTICK_MAXCOUNT ((1L<<24)-1) /* we use Systick to complete function Delay10uS(). This value only applies to ADICUP3029 board. */
#define SYSTICK_CLKFREQ   26000000L   /* Systick clock frequency in Hz. This only appies to ADICUP3029 board */
volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */
void Ext_Int0_Handler();

// void spiCommand(SPIClass *spi, byte data) {
//   //use it as you would the regular arduino SPI API
//   spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
//   digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
//   spi->transfer(data);
//   digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
//   spi->endTransaction();
// }
// void spiCommand(SPIClass *spi, byte data) {
//   spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
//   digitalWrite(spi->pinSS(), LOW);

//   // 데이터를 전송하고 리턴 값을 읽습니다.
//   byte response1 = spi->transfer(data);
//   byte response2 = spi->transfer(0);  // 더미 데이터를 보내서 리턴 값을 받습니다.

//   digitalWrite(spi->pinSS(), HIGH);
//   spi->endTransaction();

//   // 리턴 값을 처리합니다.
//   // ...
// }
/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
         provide a more efficient way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  for (unsigned long i = 0; i < length; i++) {
    unsigned char receivedData = SPI.transfer(pSendBuffer[i]);
    pRecvBuff[i] = receivedData;
  }
  //SPI 설정 종료
  SPI.endTransaction();
  


  // uint32_t tx_count=0, rx_count=0;
  // pADI_SPI0->CNT = length;
  // while(1){
  //   uint32_t fifo_sta = pADI_SPI0->FIFO_STAT;
  //   if(rx_count < length){
  //     if(fifo_sta&0xf00){//there is data in RX FIFO.
  //       *pRecvBuff++ = pADI_SPI0->RX;
  //       rx_count ++;
  //     }
  //   }
  //   if(tx_count < length){
  //     if((fifo_sta&0xf) < 8){// there is space in TX FIFO.
  //       pADI_SPI0->TX = *pSendBuffer++;
  //       tx_count ++;
  //     }
  //   }
  //   if(rx_count == length && tx_count==length)
  //     break;  //done
  // }
  // while((pADI_SPI0->STAT&BITM_SPI_STAT_XFRDONE) == 0);//wait for transfer done.
}

void AD5940_CsClr(void)
{
   digitalWrite(CS_5940,0);
}

void AD5940_CsSet(void)
{
   digitalWrite(CS_5940,1);
}

void AD5940_RstSet(void)
{
   digitalWrite(RESET_5940,1);
}

void AD5940_RstClr(void)
{
   digitalWrite(RESET_5940,0);
}
void AD5940_Delay10us(uint32_t time)
{
 
  delayMicroseconds(time*10);
  // if(time==0)return;
  // if(time*10<SYSTICK_MAXCOUNT/(SYSTICK_CLKFREQ/1000000)){
  //   SysTick->LOAD = time*10*(SYSTICK_CLKFREQ/1000000);
  //   SysTick->CTRL = (1 << 2) | (1<<0);    /* Enable SysTick Timer, using core clock */
  //   while(!((SysTick->CTRL)&(1<<16)));    /* Wait until count to zero */
  //   SysTick->CTRL = 0;                    /* Disable SysTick Timer */
  // }
  // else {
  //   AD5940_Delay10us(time/2);
  //   AD5940_Delay10us(time/2 + (time&1));
  // }
}

uint32_t AD5940_GetMCUIntFlag(void)
{
   return ucInterrupted;
}
#include <esp_system.h>
uint32_t AD5940_ClrMCUIntFlag(void)
{
  //  pADI_XINT0->CLR = BITM_XINT_CLR_IRQ0;
  //gpio_intr_status_clear(GPIO_INTERRUPT32);
  //digitalWrite(AD5940_ISR,HIGH); 
  ucInterrupted = 0;
  return 1;
}

/* This function is used to set Dn on Arduino shield(and set it to output) */
void Arduino_WriteDn(uint32_t Dn, BoolFlag bHigh)
{
  // if(Dn&(1<<3)) //set D3, P0.13
  // {
  //   pADI_GPIO0->OEN |=  1<<13;
  //   if(bHigh)
  //     pADI_GPIO0->SET = 1<<13;
  //   else 
  //     pADI_GPIO0->CLR = 1<<13;
  // }
  // if(Dn&(1<<4))//Set D4, P0.9
  // {
  //   pADI_GPIO0->OEN |=  1<<9;
  //   if(bHigh)
  //     pADI_GPIO0->SET = 1<<9;
  //   else 
  //     pADI_GPIO0->CLR = 1<<9;
  // }
}
/* Functions that used to initialize MCU platform */


uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  //다시 정리 한다. //P2.6-ADC3-A3-AD5940_Reset
  // /*Setup Pins P0.0-->SCLK P0.1-->MOSI P0.2-->MISO P1.10-->CS*/
  // pADI_GPIO1->CFG &=~(3<<14); /* Configure P1.10 to GPIO function */
  // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  attachInterrupt(digitalPinToInterrupt(AD5940_ISR), Ext_Int0_Handler, FALLING);
  AD5940_CsSet(); /* AD5941 Chip을 선택한다. */
  AD5940_Delay10us(150 * 100);
  AD5940_RstSet();
  // /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  // pADI_GPIO0->PE = 0xFFFF;
  // pADI_GPIO1->PE = 0xFFFF;
  // pADI_GPIO2->PE = 0xFFFF;
  // pADI_GPIO2->OEN |= (1<<6); //P2.6-ADC3-A3-AD5940_Reset
  // pADI_GPIO2->SET = 1<<6; //Pull high this pin.

  // /*Setup Pins P0.0-->SCLK P0.1-->MOSI P0.2-->MISO P1.10-->CS*/
  // pADI_GPIO0->CFG = (1<<0)|(1<<2)|(1<<4)|(pADI_GPIO0->CFG&(~((3<<0)|(3<<2)|(3<<4))));
  // pADI_GPIO1->CFG &=~(3<<14); /* Configure P1.10 to GPIO function */
  // pADI_GPIO1->OEN |= (1<<10); /* P1.10 Output Enable */
  // /*Set SPI Baudrate = PCLK/2x(iCLKDiv+1).*/
  // pADI_SPI0->DIV = 0;/*Baudrae is 13MHz*/
  // pADI_SPI0->CTL = BITM_SPI_CTL_CSRST|        // Configure SPI to reset after a bit shift error is detected
  //     BITM_SPI_CTL_MASEN|                   // Enable master mode
  //     /*BITM_SPI_CTL_CON|*/                     // Enable continous transfer mode
  //        BITM_SPI_CTL_OEN|                     // Select MISO pin to operate as normal -
  //           BITM_SPI_CTL_RXOF|                    // overwrite data in Rx FIFO during overflow states
  //              /*BITM_SPI_CTL_ZEN|*/                     // transmit 00 when no valid data in Tx FIFO
  //                 BITM_SPI_CTL_TIM|                     // initiate trasnfer with a write to SPITX
  //                    BITM_SPI_CTL_SPIEN;                  // Enable SPI. SCLK idles low/ data clocked on SCLK falling edge
  // pADI_SPI0->CNT = 1;// Setup to transfer 1 bytes to slave
  // /* Step2: initialize GPIO interrupt that connects to AD5940's interrupt output pin(Gp0, Gp3, Gp4, Gp6 or Gp7 ) */
  // pADI_GPIO0->IEN |= 1<<15;// Configure P0.15 as an input

  // pADI_XINT0->CFG0 = (0x1<<0)|(1<<3);//External IRQ0 enabled. Falling edge
  // pADI_XINT0->CLR = BITM_XINT_CLR_IRQ0;
  // NVIC_EnableIRQ(XINT_EVT0_IRQn);		  //Enable External Interrupt 0 source.
  
  // AD5940_CsSet();
  // AD5940_RstSet();
  return 0;
}

/* MCU related external line interrupt service routine */
void IRAM_ATTR Ext_Int0_Handler()
{
  ucInterrupted = 1;
  //Serial.println("Interrupt detected!");
  // /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
}

