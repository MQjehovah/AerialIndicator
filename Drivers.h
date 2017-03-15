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
#define NRF905                               //ʹ��nRF905
//#define USART                              //ʹ�ô���
//#define USE_WATCHDOG                       //ʹ�ÿ��Ź�
#define USE_ADC                              //ʹ��ADC
#define USE_TIMER
#define USE_TIMERA0
#define USE_TIMERA1                         
#define USE_FLASH                           //ʹ��FLASH�洢
#define USE_EXIT                            //ʹ���ⲿ�ж�
#define USE_SOFT_WDG                        //ʹ��������Ź�

#define DEF_In              6               //Ĭ����С��������������ֵ
#define DEF_DELT_I          120             //Ĭ�ϵ���ͻ��Խ����ֵ����·ͻ�������
#define DEF_MAX_I           500             //Ĭ�Ϲ�������ֵ
#define DEF_STEADY_I        10              //Ĭ���ȶ���ֵ���仯����֮����Ϊ�ȶ�������
#define DEF_PUL_MIN_I       10              //�ӵ�������С�����仯
#define DEF_PUL_MAX_I       10              //�ӵ������������仯
#define DEF_REPORT_TIME     900             //Ĭ���ϱ�ʱ�䣨��λ�룩
#define SEND_TIMES          32              //Ĭ���ϱ�����
#define DEF_ERROR_RSTTIME   720             //Ĭ�Ϲ��ϸ���ʱ�䣨��λ���ӣ�
#define DEF_VBAT_MIN        340             //Ĭ�ϵ�ص�ѹ����ֵ����λ����
#define DEF_TF_MAX          3000            //Ĭ�϶�·ͻ��ʱ�䣨��λms��
#define DEF_T1_MAX          200             //Ĭ��T1������ʱ��
#define DEF_T2_MAX          800             //Ĭ��T2������ʱ��
#define DEF_T3_MAX          200             //Ĭ��T3������ʱ��
#define DEF_T4_MAX          1050            //Ĭ��T4������ʱ��
#define DEF_CHZ_TIME        200             //Ĭ���غ�բʱ�䣨��λms��

#define DEF_LED_DELAY       500             //LED������ʱʱ��
#define DEF_SEND_DELAY      150             //���ķ�����ʱʱ��
#define DEF_SAMPALE_DELAY   5000            //����������ʱ
#define DEF_RX_DELAY        1000            //905����״̬����ʱ��
#define DEF_PWRDN_DELAY     3000            //905�ض�

#define STORAGE_START_ADDR  0x1000          //���ݴ洢��Flash��ƫ�Ƶ�ַ

#define DEF_SOFT_WDG_TIME   3000            //������Ź�����ֵ
#define DEF_SOFT_VERSION    0x0300          //����汾

#define nRF905_ADDR         0x55,0x6D,0xB3,0x11     //905���յ�ַ 







/* ���ƶ����궨�� */
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

#define LED_ON    P1OUT |= BIT0     //����
#define LED_OFF   P1OUT &= ~BIT0    //�ص�
#define LED_CONV  P1OUT ^= BIT0     //��ת


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
    unsigned char SampleReady: 1;         //����ֵ����
    unsigned char RequireTemp: 1;         //�����¶Ȳ���
    unsigned char RequireVBAT: 1;         //�����ص�ѹ����
    unsigned char VBATSample: 1;          //���ڽ��е�ص�ѹAD�����������������������
    unsigned char Sample_EN: 1;           //����ʹ��
    unsigned char SendCount;              //���ʹ���
    unsigned char NRF_Recived: 1;         //905���ձ�־
    unsigned char Error;                  //���ϱ�־(1�Ƕ�·��2�ǽӵأ�3�ǹ���ֵ)
    unsigned char usart_send:1;           //�Ƿ񴮿ڷ�������
	unsigned char TempSymbol:1;
	
	unsigned char jiedi_min_ok:1;         //��⵽�ӵ���������
	unsigned char jiedi_max_ok:1;         //��⵽�ӵ����ߵ���
	
	unsigned char Soft_WDG_EN: 1;          //������Ź�ʹ�ܱ�־λ
	unsigned char TIMCNT_EN:1;            //ʱ�����ʹ��
	
	unsigned char PowerDown:1;            //����״̬λ
//  unsigned char communication:1;        //˫��ͨ�ű�־λ
//  unsigned char ShortError: 1;          //��·����
//  unsigned char GroundError: 1;         //�ӵع���
    unsigned char jiedi;                  //�ӵؼ��״̬��־
    unsigned char VBAT_Low:1;             //��ص�ѹ�ͱ�־
    
    unsigned char ground;                 //�ӵ�״̬
    
    unsigned char nrf905_mode;            //905״̬��0Ϊ����ģʽ��1Ϊ����ģʽ��2Ϊ����ģʽ��3Ϊ����ģʽ��
    
    unsigned char LED_Count;              //LED���ƴ���

    unsigned char SampleMode;             //����ģʽ��1Ϊ1ms����һ�Σ�5Ϊ5ms����һ��,10Ϊ10ms����һ�Σ�
    
	
    
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
   #define CPU_F ((double)1000000)//CPU��Ƶ MCLK=8MHz 
#endif

#ifdef CPU_8MHZ
   #define CPU_F ((double)8000000)//CPU��Ƶ MCLK=8MHz 
#endif

#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) //΢�� 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0)) //���� 

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
