/*******************************************************************************
  * @file                   nRF905.h
  * @Author:                MQjehovah                 mail:MQjehovah@hotmail.com
  * @version                1.0.0
  * @date                   2016.4.7
  * @brief                  
  ******************************************************************************
  * @attention
*******************************************************************************/
#ifndef  _nRF905_H
#define  _nRF905_H
/* Includes ------------------------------------------------------------------*/
#include "msp430x21x2.h"
#include "Drivers.h" 
/* Definition ----------------------------------------------------------------*/
#define  uchar   unsigned char
#define  uint    unsigned int
#define  uclong  unsigned long
/* TXEN，TRX_CE，PWR_0 为收发模式控制端口 */
#define  TXEN_0     P3OUT &=~BIT6          //输出0
#define  TXEN_1     P3OUT |= BIT6          //输出1

#define  TRX_CE_0   P3OUT &=~BIT7
#define  TRX_CE_1   P3OUT |= BIT7
 
#define  PWR_0      P3OUT &=~BIT5
#define  PWR_1      P3OUT |= BIT5
/* 主入从出 */
#define  MISO_0     P3OUT &=~BIT2      
#define  MISO_1     P3OUT |= BIT2         
/* 主出从入 */
#define  MOSI_0     P3OUT &=~BIT1         
#define  MOSI_1     P3OUT |= BIT1        
/* SPI时钟端口 */
#define  SCK_0      P3OUT &=~BIT3         
#define  SCK_1      P3OUT |= BIT3         
/* SPI使能端口 */
#define  CSN_0      P3OUT &=~BIT0
#define  CSN_1      P3OUT |= BIT0


/* AM地址匹配 */
#define  AM_0       P1OUT &=~BIT7       
#define  AM_1       P1OUT |= BIT7
/* DR数据接收状态 */
#define  DR_0       P2OUT &=~BIT3
#define  DR_1       P2OUT |= BIT3
/* CD载波侦听状态 */
#define  CD_0       P2OUT &=~BIT5
#define  CD_1       P2OUT |= BIT5

/* NRF905:SPI指令 */
#define WC		0x00
#define RC		0x10
#define WTP		0x20
#define RTP		0x21
#define WTA		0x22
#define RTA		0x23
#define RRP		0x24

#define TxRxBuf_Len 14       //32字节发送数据收发缓冲区
/* Exported Functions --------------------------------------------------------*/
void nRF905Init(void);
void TxPacket(char *data_Bufer);
void RxPacket(void)	;
void SetTxMode(void);
void SetRxMode(void);
unsigned char CheckDR(void);
void RX(void);
void RX_Proc();
void nRF905_send();
void nRF905_Report(unsigned char Report_num ,INT16U Report_data,unsigned char mode);
//void nRF905_Tx_set(unsigned char num);
INT16U RX_Recive(unsigned char mode);
void Tx_clear();
#endif
/*********************************END OF FILE**********************************/
