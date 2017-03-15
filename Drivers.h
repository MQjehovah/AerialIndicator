/*******************************************************************************
  * @file                   Drivers.h
  * @Author:                MQjehovah                 mail:MQjehovah@hotmail.com
  * @version                1.0.0
  * @date                   2016.4.16
  * @brief
  ******************************************************************************
  * @attention
*******************************************************************************/
#ifndef  _DRIVERS_H
#define  _DRIVERS_H

/* Includes ------------------------------------------------------------------*/
#include  <msp430x21x2.h>
/* Definition ----------------------------------------------------------------*/
#define CPU_8MHZ
#define NRF905                               //使用nRF905
//#define USART                              //使用串口
//#define USE_WATCHDOG                       //使用看门狗
#define USE_ADC                              //使用ADC
#define USE_TIMER
#define USE_TIMERA0
#define USE_TIMERA1                         
#define USE_FLASH                           //使用FLASH存储
#define USE_EXIT                            //使用外部中断
#define USE_SOFT_WDG                        //使用软件看门狗

#define DEF_In              6               //默认最小正常工作电流阈值
#define DEF_DELT_I          120             //默认电流突变越线阈值（短路突变电流）
#define DEF_MAX_I           500             //默认过负荷阈值
#define DEF_STEADY_I        10              //默认稳定阈值（变化在这之内视为稳定电流）
#define DEF_PUL_MIN_I       10              //接地脉冲最小电流变化
#define DEF_PUL_MAX_I       10              //接地脉冲最大电流变化
#define DEF_REPORT_TIME     900             //默认上报时间（单位秒）
#define SEND_TIMES          32              //默认上报次数
#define DEF_ERROR_RSTTIME   720             //默认故障复归时间（单位分钟）
#define DEF_VBAT_MIN        340             //默认电池电压低阈值（单位伏）
#define DEF_TF_MAX          3000            //默认短路突变时间（单位ms）
#define DEF_T1_MAX          200             //默认T1最大持续时间
#define DEF_T2_MAX          800             //默认T2最大持续时间
#define DEF_T3_MAX          200             //默认T3最大持续时间
#define DEF_T4_MAX          1050            //默认T4最大持续时间
#define DEF_CHZ_TIME        200             //默认重合闸时间（单位ms）

#define DEF_LED_DELAY       500             //LED闪灯延时时间
#define DEF_SEND_DELAY      150             //报文发送延时时间
#define DEF_SAMPALE_DELAY   5000            //采样休眠延时
#define DEF_RX_DELAY        1000            //905接收状态保持时间
#define DEF_PWRDN_DELAY     3000            //905关断

#define STORAGE_START_ADDR  0x1000          //数据存储在Flash中偏移地址

#define DEF_SOFT_WDG_TIME   3000            //软件看门狗重载值
#define DEF_SOFT_VERSION    0x0300          //软件版本

#define nRF905_ADDR         0x55,0x6D,0xB3,0x11     //905接收地址 







/* 翻牌动作宏定义 */
#define TDI_H P1OUT   |=  BIT6
#define TDI_L P1OUT   &= ~BIT6

#define TDO_H P1OUT   |=  BIT7
#define TDO_L P1OUT   &= ~BIT7

#define ENVD_H P1OUT  |=  BIT3
#define ENVD_L P1OUT  &= ~BIT3

#define WHT_H P1OUT   |=  BIT2
#define WHT_L P1OUT   &= ~BIT2

#define TURNE_H P1OUT |=  BIT1
#define TURNE_L P1OUT &= ~BIT1

#define LED_ON    P1OUT |= BIT0     //亮灯
#define LED_OFF   P1OUT &= ~BIT0    //关灯
#define LED_CONV  P1OUT ^= BIT0     //翻转


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef unsigned char BOOLEAN;
typedef unsigned char INT8U;   //unsigned 8-bit quantity
typedef          char INT8S;   //signed  8-bit quantity
typedef unsigned int  INT16U;  //unsigned 16-bit quantity
typedef          int  INT16S;  //signed 16-bit quantity
typedef unsigned long INT32U;  //unsigned 32-bit quantity
typedef          long INT32S;  //signed  32-bit quantity
typedef   float       FP32;    //single precsion floating point
typedef   double      FP64L;   //double precision floating pont

typedef struct _flagbit
{
    unsigned char SampleReady: 1;         //采样值就绪
    unsigned char RequireTemp: 1;         //请求温度采样
    unsigned char RequireVBAT: 1;         //请求电池电压采样
    unsigned char VBATSample: 1;          //正在进行电池电压AD采样（用于区别电流采样）
    unsigned char Sample_EN: 1;           //采样使能
    unsigned char SendCount;              //发送次数
    unsigned char NRF_Recived: 1;         //905接收标志
    unsigned char Error;                  //故障标志(1是短路，2是接地，3是过阈值)
    unsigned char usart_send:1;           //是否串口发送数据
	unsigned char TempSymbol:1;
	
	unsigned char jiedi_min_ok:1;         //检测到接地起跳电流
	unsigned char jiedi_max_ok:1;         //检测到接地跳高电流
	
	unsigned char Soft_WDG_EN: 1;          //软件看门狗使能标志位
	unsigned char TIMCNT_EN:1;            //时间计数使能
	
	unsigned char PowerDown:1;            //掉电状态位
//  unsigned char communication:1;        //双向通信标志位
//  unsigned char ShortError: 1;          //短路故障
//  unsigned char GroundError: 1;         //接地故障
    unsigned char jiedi;                  //接地检测状态标志
    unsigned char VBAT_Low:1;             //电池电压低标志
    
    unsigned char ground;                 //接地状态
    
    unsigned char nrf905_mode;            //905状态（0为掉电模式，1为休眠模式，2为发送模式，3为接收模式）
    
    unsigned char LED_Count;              //LED闪灯次数

    unsigned char SampleMode;             //采样模式（1为1ms采样一次，5为5ms采样一次,10为10ms采样一次）
    
	
    
} flagbit;

struct _storage_data
{
    INT16U Soft_Version;
    INT32U FLASH_REPORT_TIME;
    INT32U FLASH_ERROR_RSTTIME;
    INT16U FLASH_TF_MAX;
    INT16U FLASH_DELT_I;
    INT16U FLASH_CHZ_TIME;
    INT16U FLASH_MAX_I;
    INT16U FLASH_T1_MAX;
    INT16U FLASH_T2_MAX;
    INT16U FLASH_T3_MAX;
    INT16U FLASH_T4_MAX;
    INT16U FLASH_VBAT_MIN;
    INT16U FLASH_SAMPLE_TABLE[14];
};

#ifdef CPU_1MHZ
   #define CPU_F ((double)1000000)//CPU主频 MCLK=8MHz 
#endif

#ifdef CPU_8MHZ
   #define CPU_F ((double)8000000)//CPU主频 MCLK=8MHz 
#endif

#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) //微秒 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0)) //毫秒 

/* Exported Functions --------------------------------------------------------*/
/* LED */
void led_init( void );
void led_flash(INT16U time, INT8U n );
/* Flop */
void flop_init( void );
void fanhong();
void fanbai();
/* Timer */
void TimerA0_Init( void );
void TimerA1_Init( void );
void Start_Timer_A0( void );
void Start_Timer_A1( void );
void Stop_Timer_A0( void );
void Stop_Timer_A1( void );
/* ADC */
INT16U SampleChannel( INT16U ChannelNox );
void adc_init();
INT16U ADC_test();
INT16U adc_get_value( unsigned char channel );
//INT16U adc_get_sample( unsigned char channel );
INT16U SingleSample(unsigned char channel);
void wakeAD();
/* USART */
void usart_init( void );
void UartPutchar( unsigned char c );
unsigned char UartGetchar();
void print( char* str );
void println( char* str );
void printNumber( INT16U n );
void printNumber1( unsigned long num, unsigned char base );
//void bubble(INT16U *a,int n);
/* FLASH */
void FLASH_LOAD();
void FLASH_SAVE();
void Flash_Clr(INT16S *Data_ptr);

void statuscheck( INT16U value );

void gpio_init( void );

void compare_init();
void exit_init();



void statuscheck1( float value );


void k_cacl(void);
void Table1_50_cacl(void);
#endif
/*********************************END OF FILE**********************************/
