/*******************************************************************************
  * @file                   Drivers.c
  * @Author:                MQjehovah                 mail:MQjehovah@hotmail.com
  * @version                1.0.0
  * @date                   2016.4.16
  * @brief
  ******************************************************************************
  * @attention
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "Drivers.h"
#include "nRF905.h"
#include <math.h>

#ifdef USART
#include "stdio.h"
#endif
/* Definition ----------------------------------------------------------------*/
union     //flash���ݴ洢������
{                              
  unsigned char Storage[sizeof(struct _storage_data)];          
  struct _storage_data StorageData;               
}Storage_union;

/* ָ����ַ 
const INT32U FLASH_REPORT_TIME @ 0x1000  = DEF_REPORT_TIME * 60000; //�ϱ�ʱ�䣨ң�ⷢ�ͼ����
const INT32U FLASH_ERROR_RSTTIME @ 0x1004  = DEF_ERROR_RSTTIME * 3600000; //���ϸ���ʱ��
const INT16U FLASH_TF_MAX @ 0x1008 = DEF_TF_MAX/10;          //��·ͻ��ʱ��
const INT16U FLASH_DELT_I @ 0x100A = DEF_DELT_I;             //����ͻ��Խ����ֵ����·ͻ�������

const INT16U FLASH_MAX_I @ 0x100C = DEF_MAX_I;               //��������ֵ
const INT16U FLASH_T1_MAX @ 0x100E= DEF_T1_MAX/10;               //����10��ÿ10msһ�Σ�
const INT16U FLASH_T2_MAX @ 0x1010= DEF_T2_MAX/10;
const INT16U FLASH_T3_MAX @ 0x1012= DEF_T3_MAX/10;
const INT16U FLASH_T4_MAX @ 0x1014= DEF_T4_MAX/10;
const INT16U FLASH_VBAT_MIN @ 0x1016= DEF_VBAT_MIN; //��ص�ѹ����ֵ
*/ 
/* ������� */
INT32U Report_time = DEF_REPORT_TIME * (INT32U)1000; //�ϱ�ʱ�䣨ң�ⷢ�ͼ����
INT32U ERROR_RSTTIME = DEF_ERROR_RSTTIME * 60000; //���ϸ���ʱ��(��λ����)
INT16U TF_MAX = DEF_TF_MAX/10;          //��·ͻ��ʱ��
INT16U DELT_I = DEF_DELT_I;             //����ͻ��Խ����ֵ����·ͻ�������
INT16U CHZ_TIME=DEF_CHZ_TIME/10;        //�غ�բʱ��
INT16U MAX_I = DEF_MAX_I;               //��������ֵ
INT16U T1_MAX = DEF_T1_MAX/10;          //�ӵ�����1ʱ�䣨����10��ÿ10msһ�Σ���
INT16U T2_MAX = DEF_T2_MAX/10;          //�ӵؼ��1ʱ��
INT16U T3_MAX = DEF_T3_MAX/10;          //�ӵ�����2ʱ��
INT16U T4_MAX = DEF_T4_MAX/10;          //�ӵؼ��2ʱ��
INT16U VBAT_MIN = DEF_VBAT_MIN;         //��ص�ѹ����ֵ


//{0xfc,     0xf5,       0xe2,       0xef,       0xef,       0xf0,       0xf4,       0xfa,     0x03,       0x0a,     0x22    ,   0x05};
//--------------------0~50--50~80--80~100--100~150--150~200--200~250--250~300--300~350--350~400--400~500--500~600--600~630

/* �ڲ����� */
INT16U STEADY_I = DEF_STEADY_I;         //�ȶ���ֵ
INT16U last_i, cur_i, normal_i, If;     //��ǰ�������ϴε���
INT16U pre_i;
unsigned char status;                   //״̬��־
long  In_t;                             //������ʱ

INT16U delt_t, Tf, Ts,Tc;               //״̬����

INT32U ms_count;                        //ʱ�����

INT32U RstTime_Count = 0;               //��ʱ�����ʱ
INT16U Error_FlashTime=0;               //�й���ʱ����ʱ�����
unsigned char f_steady;                 //�ȶ���־

//�ӵ�����
INT16U PUL_MIN_I = DEF_PUL_MIN_I;       //��С�ӵ�����
INT16U PUL_MAX_I = DEF_PUL_MAX_I;       //���ӵ�����

INT16U maichong_i;                      //����ο���׼ֵ
INT16U Groundsteady;                    //�ӵ��ȶ���ֵ

INT16U LED_TimeCount;                   //������ʱ����

INT16U T1, T2, T3, T4;                  //�ӵ�����ʱ�����
unsigned char num;                      //�ӵ��������

INT16U Id,Ig;                           //�ӵع��ϵ���

INT16U T_zero;                          //ʧ��ʱ�����
INT16U Tm;                              //����ֵʱ�����             
INT16U CPU_TEMP;                        //CPU�¶�ֵ         
INT16U VBAT;                            //��ص�ѹֵ

INT32U SampleBuffer[10];                //����������

unsigned char SampleCount = 0;          //����ֵָ��
INT32U SampleValue;                     //����ֵ
INT32U oldSampleValue;                  //�ϴβ���ֵ


INT16U LED_DELAY=DEF_LED_DELAY;         //LED��˸��ʱ
INT16U SEND_DELAY;                      //905������ʱ
INT16U PWRDN_DELAY;                     //ϵͳ������ʱ
INT32U RX_DELAY;                        //905������ʱ
INT32U PRE_RX_DELAY=DEF_RX_DELAY;       //905������ʱʱ��Ԥ��ֵ����ÿ����Ϊ����ģʽ�������װ�ش�ֵ��Ϊ��ʱʱ�䣩

INT32U SingleSample_count;              //���β�������ص�ѹ��CPU�¶Ȳ�������ʱ����
INT32U Report_time_count;               //��ʱ�ϱ���ʱ����

INT16U SampleTemp;                      //���β�����ʱ��ű���
INT16U jiedi_delay;                     //��ֹ����������иı�ο�����ֵ

flagbit flag;                           //����ϵͳ��־λ����

INT16U SampleDelay=DEF_SAMPALE_DELAY;   //������ʱ

INT16U JUGE_DELT_I;                     //��Ϊ�оݵĵ���ͻ����ֵ

INT16U SOFT_WDG=65535;

extern char TxBuf[TxRxBuf_Len];
extern INT16U SampleTable[14];


INT16U * TIMCNT_Ptr;
//INT16U tam[10];
/* Functions -----------------------------------------------------------------*/
#ifdef USE_EXIT
/*******************************************************************************
  * @brief  �ⲿ�жϳ�ʼ��
  * @param  None
  * @retval None
  * @Note   P2.5�˿ڽӱȽ�������ˣ�
            ����ֵ��5~70���Ƚ��������һ�����������ֵ�仯�ķ���
*******************************************************************************/
void exit_init()
{
    P2DIR &= ~BIT5;
    //P2REN |= BIT5;
    //P2OUT |= BIT5;
    P2IE |= BIT5; 
    //P2IES |= BIT5;
    P2IFG &= ~BIT5;                //P2.5 input interrupt configurate
}
//P2���ⲿ�жϷ�����
#pragma vector =PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
  if( P2IFG & BIT5 )
  {
     // wakeAD();
     if(flag.SampleMode==0)flag.SampleMode=5;
     SampleDelay=DEF_SAMPALE_DELAY;
    //LED_CONV;
  }
 P2IFG &= ~BIT5;  
}
#endif

#ifdef COMPARE
/*******************************************************************************
  * @brief  �ڲ��Ƚ�����ʼ��
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void compare_init()
{
    CACTL1 |= CAREF_1 + CAON + CAREF_2  /*+ CAIE */;          // 0.25 Vcc On -comp, (�ο���ѹ���ڱȽ�������)
    //CACTL2 |= CAF;
    CACTL2 = P2CA1 + P2CA3;              //ʹ��CA5

}

/*******************************************************************************
  * @brief  �Ƚ����ж�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=COMPARATORA_VECTOR
__interrupt void Comparator_A( void )
{
#ifdef USART
    //printNumber(1);
#endif
}
#endif

/*******************************************************************************
  * @brief  ��������
  * @param  None
  * @retval None
  * @Note   ����δʹ�����ŷ�ֹ���յ�����ʵ��200uA��
*******************************************************************************/
void gpio_init( void )
{
    //P1DIR |= BIT4 + BIT5 ;
    //P1OUT &= ~(BIT4 + BIT5 + BIT6 + BIT7);
    //P2DIR |=BIT0+BIT1+ BIT2 + BIT4;

    //P2OUT &= ~(BIT2 + BIT4);
    
    //P1REN = 0xFF;
    P1OUT = 0;
    P1DIR = 0xFF;
    //P2REN = 0xFF;
    P2OUT = 0;
    P2DIR = 0xFF;
    //P3REN = 0xFF;
    P3OUT = 0;
    P3DIR = 0xFF;
    
    //P2DIR &= ~BIT1;
}



/*******************************************************************************
  * @brief  ������ȡ����ֵ
  * @param  None
  * @retval None
  * @Note   ����������󣬿����Ż�
*******************************************************************************/
float absf( float value )
{
    if ( value < 0 )
    {
        value = -value;
    }
    return value;
}
/*******************************************************************************
  * @brief  INT16Uȡ����ֵ
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
INT16U abs( INT16U a,INT16U b )
{
    INT16S x = (INT16S)a;
    INT16S y = (INT16S)b;  
    INT16S num = x-y;
    if ( num < 0 )
    {
        num = -num;
    }
    return (INT16U)num;
}
/*******************************************************************************
  * @brief  �ж�������
  * @param  None
  * @retval None
  * @Note   �������Դ�������֧��������
*******************************************************************************/
INT16S RiseEdge( INT16U a,INT16U b )
{
    INT16S x = (INT16S)a;
    INT16S y = (INT16S)b;  
    INT16S num = x-y;

    return num;
}

/*******************************************************************************
  * @brief  LED��ʼ��
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void led_init( void )
{
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;
}

/*******************************************************************************
  * @brief  LED��˸ 1s��ʱ
  * @param  n��LED��˸����
            time:��ʱʱ�䣬��λms
  * @retval None
  * @Note   None
*******************************************************************************/
void led_flash(INT16U time, INT8U n )
{
    /*
    unsigned char i;
    for ( i = 0; i < n; i++ )
    {
        P1OUT |= BIT0;
        delay_ms( 1000 );
        P1OUT &= ~BIT0;
        delay_ms( 1000 );
    }
    */
    LED_DELAY=time;
    flag.LED_Count+=n*2;
}


/*******************************************************************************
  * @brief  ���Ƴ�ʼ��
  * @param  None
  * @retval None
  * @Note   ����P1��1��2��3��6��7����Ϊ����˿�
*******************************************************************************/
void flop_init( void )
{
    P1DIR |= BIT1 + BIT2 + BIT3 + BIT6 + BIT7;
}

/*******************************************************************************
  * @brief  ����
  * @param  None
  * @retval None
  * @Note   ��ѹ��·
*******************************************************************************/
void fanhong()
{
    WHT_L;
    TURNE_L;
    TDI_L;
    TDO_H;
    ENVD_H;
    delay_ms( 100 );
    TDI_H;
    TDO_L;
    ENVD_L;
    delay_ms( 100 );
    ENVD_H;
    WHT_H;
    delay_ms( 100 );
    WHT_L;
}

/*******************************************************************************
  * @brief  ����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void fanbai()
{
    WHT_L;
    TURNE_L;
    TDI_L;
    TDO_H;
    ENVD_H;
    delay_ms( 100 );
    TDI_H;
    TDO_L;
    ENVD_L;
    delay_ms( 100 );
    ENVD_H;
    TURNE_H;
    delay_ms( 100 );
    TURNE_L;
}




/************************************��ʱ��************************************/



#ifdef USE_TIMER

#ifdef USE_TIMERA0
/*******************************************************************************
  * @brief  ��ʱ��A0��ʼ��
  * @param  None
  * @retval None
  * @Note   ʱ��ѡ��ACLK��������ģʽ�������ֵ
*******************************************************************************/
void TimerA0_Init( void )
{
    TA0CCR0 = 327;                    //��ʱ10ms
    //TA0CCR0 = 164;                  //��ʱ5ms
    //TA0CCR0 = 16384;
    //TA0CCR0 = 32768;
    TA0CTL  = TASSEL_1 + MC_1;        //ACLKʱ��
}

/*******************************************************************************
  * @brief  ������ģʽ�£���ʼ��ʱ
  * @param  None
  * @retval None
  * @Note   �����ж�
*******************************************************************************/
void Start_Timer_A0( void )
{
    TA0CTL |= TACLR;
    TA0CCTL0 |= CCIE ;   //������ʱ��
    _EINT();
}

/*******************************************************************************
  * @brief  ������ģʽ�£�ֹͣ��ʱ
  * @param  None
  * @retval None
  * @Note   �ر��ж�
*******************************************************************************/
void Stop_Timer_A0( void )
{
    TA0CCTL0 &= ~CCIE ;               //ֹͣģʽ
}

/*******************************************************************************
  * @brief  ��ʱ��A0�ж�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 ( void )
{
   // int i;
    /* ���ÿ��Ź� */
#ifdef USE_WATCHDOG
    WDTCTL = WDTPW + WDTCNTCL;
#endif
/* ʱ����� */
    ms_count+=10;
    if(flag.Error>0)        //��ʱ����
    {
        RstTime_Count+=10;
		Error_FlashTime++;
        if(Error_FlashTime>300)//1sһ����ȡģ������д�������ʵ��10uA��
        {
            led_flash(100,1);
        }
        if(RstTime_Count>ERROR_RSTTIME)
        {
            RstTime_Count=0;
            fanbai();              //���飨���ƣ�
            nRF905_Report(0x17,0,0);
            flag.Error = 0;        //��������
        }
        
    }

    if ( ms_count >= 86400000 ) //��ʱ��ﵽ1��
    {
        ms_count = 0;
    }


    
    
    
/* LED��ʱ��˸ */ 
    if(flag.LED_Count>0)
    {
        LED_TimeCount+=10;
        if(LED_TimeCount==LED_DELAY)
        {
            LED_TimeCount=0;
            LED_CONV;
            flag.LED_Count--;
        }

    }

    
 
/* 905���ձ���  */
    if(RX_DELAY>0)
    {
        RX_DELAY-=10;
    }    
    else if(flag.nrf905_mode==3)
    {
        TRX_CE_0;
        //PWR_0;
        flag.nrf905_mode=1; //����ģʽ
    }

    if(flag.nrf905_mode==1 )
    {
        if( PWRDN_DELAY>0 )
        {
            PWRDN_DELAY-=10;
        }
        else
        {
            PWR_0;
            PRE_RX_DELAY=DEF_RX_DELAY;//����������ʱԤ��ֵ
            flag.nrf905_mode=0;//����ģʽ
        }
        
    }

 
/* ���ķ�����ʱ  */
    if(SEND_DELAY>0)
    {
        SEND_DELAY-=10;

    }

 
/* 1s��ʱ��ѹ���� */
    SingleSample_count+=10;
    if ( SingleSample_count >= 60000 ) //10S�ж�
    {
        SingleSample_count=0;
        flag.RequireTemp=1; //CPU�¶Ȳ���
        if(flag.VBAT_Low==0)
        {
            flag.RequireVBAT=1; //��ص�ѹ����
        }
    }
/* ȡģ �������Ĵ�
    if ( ms_count % 60000 == 0 ) //10S�ж�
    {
        led_flash(100,1);
       if(flag.VBAT_Low==0)
        {
          flag.RequireVBAT=1; 
        }
           flag.RequireVBAT=1; 
  
    }
*/


    if(SampleDelay>0)
    {
         if((P2IN&0x20)==0)SampleDelay-=10;//�������ߣ���ϵͳ���ߣ��ڱȽ��������ʱһֱ�����ֵ��һ������<5A�Ƚ���ֹͣ�������ֵ�ήΪ0��ϵͳ�������ߣ�
    }



/* ��ʱ�ϱ� */
    Report_time_count+=10;
    if ( Report_time_count >= Report_time)
    {
        Report_time_count=0;
#ifdef NRF905
        

        TxBuf[8]=VBAT/256;
        TxBuf[9]=VBAT%256;
                    
        //TxBuf[10]=((INT16U)(CPU_TEMP))/256;
        //TxBuf[11]=((INT16U)(CPU_TEMP))%256;    
        TxBuf[10]=flag.TempSymbol;
		TxBuf[11]=((INT8U)(CPU_TEMP));		
        TxBuf[13]=flag.Error;
        
        nRF905_Report(0x30,(INT16U)cur_i,0);
#endif     
    }
 
#ifdef USE_SOFT_WDG
if(flag.Soft_WDG_EN==1)
{
	if(SOFT_WDG>0)
	{
		if(flag.SampleMode!=0)
			SOFT_WDG--;
	}
	else //�����λ
	{
		SOFT_WDG=DEF_SOFT_WDG_TIME;
		WDTCTL = WDTCTL|0xff00;//
	}
}
#endif
 
 
if(flag.TIMCNT_EN==1)
{
	(*TIMCNT_Ptr)++;
}

}
#endif


#ifdef USE_TIMERA1
/*******************************************************************************
  * @brief  ��ʱ��A1��ʼ��
  * @param  None
  * @retval None
  * @Note   ʱ��ѡ��ACLK/SMCLK��������ģʽ�������ֵ
*******************************************************************************/
void TimerA1_Init( void )
{
/* 32KHz���� */
    //TA1CCR0 = 33;                   //��ʱ1ms
    TA1CCR0 = 164;                  //��ʱ5ms
    //TA1CCR0 = 16384;
    //TA1CCR0 = 327;                    //��ʱ10ms
    //TA1CCR0 = 32768;                  //��ʱ1s
    TA1CTL  = TASSEL_1 + MC_1;        //ACLKʱ��

/* 8M���� 
    TA1CCR0 = 5000;
    TA1CTL  = TASSEL_2 + MC_1 + ID_3; //ѡ��SMCLK 8��Ƶ
*/
}

/*******************************************************************************
  * @brief  ������ģʽ�£���ʼ��ʱ
  * @param  None
  * @retval None
  * @Note   �����ж�
*******************************************************************************/
void Start_Timer_A1( void )
{
    //TA1CTL  = TASSEL_1 + MC_1;        //ACLKʱ��
    TA1R=0;
    TA1CTL |= TACLR;
    TA1CCTL0 |= CCIE ;   //������ʱ��
    _EINT();
}

/*******************************************************************************
  * @brief  ������ģʽ�£�ֹͣ��ʱ
  * @param  None
  * @retval None
  * @Note   �ر��ж�
*******************************************************************************/
void Stop_Timer_A1( void )
{
    TA1CCTL0 &= ~CCIE ;               //ֹͣģʽ
     //TA1CTL  = TASSEL_1 + MC_0;        //ACLKʱ��
}

/*******************************************************************************
  * @brief  ��ʱ��A1�ж�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1 ( void )
{
    /* ����AD���� */
    if((P2IN&0x20)==0x20)
	{
		if(flag.SampleMode==0)flag.SampleMode=5;
		SampleDelay=DEF_SAMPALE_DELAY;
	}
    if ( flag.Sample_EN == 1 & !(flag.SampleMode==0))   //!(SampleDelay==0 & (P2IN&0x20)==0))
    {
       //SampleValue = SingleSample(0);
        wakeAD();

    }
}

#endif

#endif



/*************************************ADC**************************************/


#ifdef USE_ADC



/*******************************************************************************
  * @brief  ADC��ʼ��
  * @param  None
  * @retval None
  * @Note   �ڲ�2.5V�ο���ѹ
            �Դ�5Mʱ��
*******************************************************************************/
void adc_init()
{
    ADC10CTL0 &= ~ENC;                                       //�ڸı�����ǰֹͣA/Dת��
    while (ADC10CTL1 & BUSY);                                //�ȴ�ADC10���ļ���
    ADC10CTL0 = SREF_1 +  REF2_5V /*+ REFON */+ ADC10SHT_3 + ADC10ON + ADC10IE +/* REFOUT */+ ADC10SR /*+ REFBURST */; // ADC10ON, interrupt enabled
    ADC10AE0 |= 1;                         // P2.0 ADC option select
    //ADC10CTL1 = CONSEQ_2;//SHS_1;
    //ADC10CTL0 |= ENC;
    //ADC10CTL0 |= 0x1<<13;       //ѡ���ڲ��ο���ѹ   SREF_1
    //ADC10CTL0 |= 0x1<<5;        //���ڲ��ο���ѹ
    //ADC10CTL0 |= 0x1<<6;        //ѡ��2.5V�ڲ��ο���ѹ  REF2_5V
}

/*******************************************************************************
  * @brief  ADC�ر�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void adc_deinit()
{
    ADC10CTL0 = 0; 
    ADC10CTL1 = 0;
}
/*******************************************************************************
  * @brief  ��ͨ�����β��������жϣ�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
INT16U adc_get_value( unsigned char channel )
{
    INT16U temp;
    
    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );

    switch ( channel )
    {
    case 0:
        ADC10CTL1 = INCH_0;
        break;
    case 1:
        ADC10CTL1 = INCH_1;
        break;
    case 10:
        ADC10CTL1 = INCH_10;
        break;
    case 11:
        ADC10CTL1 = INCH_11;
        break;
    default:
        ADC10CTL1 = INCH_0;
        break;

    }
    ADC10CTL0 |= REFON; 
    
    flag.VBATSample=1;
    
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    
    __bis_SR_register( CPUOFF + GIE );      // LPM0, ADC10_ISR will force exit
   // while (ADC10CTL1 & ADC10BUSY);               // ADC10BUSY?
    temp=SampleTemp;
    //adc_init();
    return temp;
}

/*******************************************************************************
  * @brief  ����AD
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void wakeAD()
{
    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );
//  ADC10AE0 |= 3;                         // P2.0 ADC option select

    ADC10CTL0 |= REFON;                     //�����ڲ���׼��ѹ
    
    ADC10CTL1 = INCH_0;

    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
}

/*******************************************************************************
  * @brief  ���β��������ж�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
INT16U SingleSample(unsigned char channel)
{
    INT16U temp;
    INT16U ctl1;                 //����ADC��ǰ�ļĴ�������
    INT16U ctl0;                 //����ADC��ǰ�ļĴ�������

    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );
    
    ctl1 =  ADC10CTL1;
    ctl0 =  ADC10CTL0;
    
    

    switch ( channel )
    {
        case 0:
            ADC10CTL0 = SREF_1 + REF2_5V + REFON + ADC10SHT_3 + ADC10ON;
            ADC10CTL1 = INCH_0;
            break;
        case 1:
            ADC10CTL0 = SREF_1 + REF2_5V + REFON + ADC10SHT_3 + ADC10ON;
            ADC10CTL1 = INCH_1;
            break;
        case 10:
            ADC10CTL0 = SREF_1 + REFON + ADC10SHT_3 + ADC10ON;
            ADC10CTL1 = INCH_10;
            break;
        case 11:
            ADC10CTL0 = SREF_1 + REF2_5V + REFON + ADC10SHT_3 + ADC10ON;
            ADC10CTL1 = INCH_11;
            break;
        default:
            ADC10CTL0 = SREF_1 + REF2_5V + REFON + ADC10SHT_3 + ADC10ON;
            ADC10CTL1 = INCH_0;
            break;
    }
    
    ADC10CTL0 |= ENC + ADC10SC;               // Sampling and conversion start
    while ( ADC10CTL1 & BUSY );
    temp=ADC10MEM;
    ADC10CTL0 &= ~(ENC + ADC10SC);  
    while ( ADC10CTL1 & BUSY );
    ADC10CTL1 = ctl1;                         //�رղ���λADC״̬
    ADC10CTL0 = ctl0;                         //�رղ���λADC״̬
    return temp;
}

/*******************************************************************************
  * @brief  ����ð���㷨������
  * @param  unsigned int *pData������Ҫ��������ָ�룺unsigned char Count�����ݸ���
  * @retval None
  * @Note   ����ֵ��С��������
******************************************************************************
void bubble(INT16U *a,int n) //�������������������׵�ַ�������С
{
    int i,j,temp;
    for(i=0;i<n-1;i++)
        for(j=i+1;j<n;j++) //ע��ѭ����������
            if(a[i]>a[j])
            {
                temp=a[i];
                a[i]=a[j];
                a[j]=temp;
            }
}
*/

/*******************************************************************************
  * @brief  ADC10�жϷ�����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR( void )
{
    
    int i = 0;
    //SampleValue =ADC10MEM;
    /* ����ǵ�ص�ѹ�������ж� 
    if ( flag.VBATSample == 1 )
    {
        flag.VBATSample=0;
        SampleTemp=ADC10MEM;
        __bic_SR_register_on_exit( CPUOFF );      // Clear CPUOFF bit from 0(SR)

    }
    */
  //else
  //{
         /* ����ֱֵ�����
        printNumber((long)ADC10MEM);
          */
  //}
          
          
          
    /* ������ֵ 
    //double temp;
    //temp = ( double )ADC10MEM;
    
    
    
    
    //SampleBuffer[SampleCount++] = pow(ADC10MEM, 2 );
    SampleBuffer[SampleCount++] =ADC10MEM;
    

    //SampleBuffer[SampleCount++] = temp*temp;
    if ( SampleCount > 9 )
    {
        //SampleValue = 0;
        SampleCount = 0;
        // ������������ 
        for ( i = 0; i < 10; i++ )
        {
            SampleValue += SampleBuffer[i];
        }

        SampleValue =sqrt( SampleValue / 10 ); //ȡ������
        //oldSampleValue=SampleValue;
        //һ���ͺ��˲�
        oldSampleValue = ( 0.5 * oldSampleValue ) + ( 0.5 * SampleValue );
       
        flag.SampleReady = 1;
        LPM3_EXIT;
        //flag.RequireVBAT=1;
    }
*/

    /* ����ƽ���˲�

       SampleBuffer[10]= ADC10MEM;
       SampleValue=0;
       for(i=0;i<10;i++)
       {
            SampleBuffer[i]=SampleBuffer[i+1];
            SampleValue += SampleBuffer[i];
       }
       SampleValue /= 10;
       //oldSampleValue=SampleValue;
       oldSampleValue=(0.9*oldSampleValue)+(0.1*SampleValue);
       flag.SampleReady=1;
    */

    /* 10��ȡƽ�� */
       // SampleValue=0.9*SampleValue+0.1*ADC10MEM;
       // printNumber((long)SampleValue);

        //SampleCount++;
       // if(ADC10MEM>SampleValue1)SampleValue1=ADC10MEM;
       // printNumber((long)ADC10MEM);
       // if(SampleCount>9)
       // {
           // flag.Sample_EN = 0;
        //    SampleValue=SampleValue1;
        //    SampleValue=0;
        //    SampleCount=0;

           // for ( i= 0; i<10; i++)
           // {
           //     SampleValue1 += SampleBuffer1[i];
           // }
            //SampleValue=sqrt(SampleValue/10);  //ȡ������
           // SampleValue1 /= 10;                   //ȡƽ��ֵ
            //һ���ͺ��˲�
            //oldSampleValue1=SampleValue1;

           // SampleValue=(SampleBuffer[0]+SampleBuffer[1])/2;
            //oldSampleValue=(0.5*oldSampleValue)+(0.5*SampleValue);
           // for(i=0;i<45;i++)
           // {
               
           // }
            
       //    flag.SampleReady=1;
            //LPM3_EXIT;
       // }
   

    /* FFT
        SampleBuffer[SampleCount++]= ADC10MEM;
        if(SampleCount>7)
        {
            SampleCount=0;
            flag.SampleReady=1;
        }
     */
   // }
    switch(flag.SampleMode)           //���ݲ�ͬ�Ĳ���ģʽ���ò�ͬ���жϷ���
    {
        case 0:                       //5ms����1��
            SampleBuffer[SampleCount++]= ADC10MEM;
            if(SampleCount>1)
            {
                SampleCount=0;
                oldSampleValue=(SampleBuffer[0]+SampleBuffer[1])/2;
                flag.SampleReady=1;
                LPM3_EXIT;
                //LPM0_EXIT;
            }
            break;
        case 1:                       //1ms����1��
            SampleBuffer[SampleCount++] = (INT32U)pow(ADC10MEM, 2 );
            if ( SampleCount > 9 )
            {
                SampleValue = 0;
                SampleCount = 0;
                // ������������ 
                for ( i = 0; i < 10; i++ )
                {
                    SampleValue += SampleBuffer[i];
                }

                SampleValue =(INT32U)sqrt( (double)(SampleValue / 10 )); //ȡ������
                //oldSampleValue=SampleValue;
                //һ���ͺ��˲�
                //oldSampleValue1 = (INT32U)(( 0.5 * oldSampleValue ) + ( 0.5 * SampleValue ));
                oldSampleValue=SampleValue;
                flag.SampleReady = 1;
                //LPM3_EXIT;
                LPM0_EXIT;
                //flag.RequireVBAT=1;
             }
            break;
        case 5:                        //5ms����1��
            SampleBuffer[SampleCount++]= ADC10MEM;
            if(SampleCount>1)
            {
                //flag.Sample_EN=0;
                SampleCount=0;
                oldSampleValue=(SampleBuffer[0]+SampleBuffer[1])/2;
               // oldSampleValue=(SampleBuffer[0]+SampleBuffer[1])>>1;
                flag.SampleReady=1;
                LPM3_EXIT;
                //LPM0_EXIT;
            }
            break;
        case 10:                       //10ms����1��
            oldSampleValue=ADC10MEM;
            //flag.Sample_EN=0;
            flag.SampleReady=1;
            LPM3_EXIT;
            //LPM0_EXIT;
            break;
       // default:
         //   SampleBuffer1[SampleCount++]= ADC10MEM;
         //   if(SampleCount>1)
        //    {
         //       SampleCount=0;
       //         oldSampleValue=(SampleBuffer1[0]+SampleBuffer1[1])/2;
        //        flag.SampleReady=1;
                //LPM3_EXIT;
       //         LPM0_EXIT;
        //    }
        //    break;
           
    }
//�ض��ڲ���ѹ��(��ѹ��һֱ�������ĺܴ�ʵ��200uA)
    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );
    ADC10CTL0 &= ~REFON;


}

/*******************************************************************************
  * @brief  ����AD����ģʽ              
  * @param  mode   0:5ms����1��
                   1:1ms����1��
  * @retval None              
  * @Note   �������û�����CPU�������Ѿ�����              
******************************************************************************
void Set_ADMode(unsigned char mode)
{
    switch(mode)
    {
        case 0:
            TA1CTL  = TASSEL_1 + MC_1;        //ACLKʱ��
            flag.SampleMode=0;
            TA1CCR0 = 164;
            //TA1CCR0 = 5000;
            break; 
        case 1:                               //1ms����1��
            TA1CTL  = TASSEL_2 + MC_1 + ID_3;        //ACLKʱ��
            flag.SampleMode=1;
            TA1CCR0 = 1000;
            break;
        case 5:                    //5ms����1��
            TA1CTL  = TASSEL_1 + MC_1;        //ACLKʱ��
            flag.SampleMode=5;
            TA1CCR0 = 164;
            //TA1CCR0 = 5000;
            break;
        case 10:
            //flag.SampleMode=10;
           // TA1CCR0 = 327;
            break;          
    }
    SampleCount=0;
}
*/ 
    
    
    
#endif


/***********************************����***************************************/



#ifdef USART

/*******************************************************************************
  * @brief  ���ڳ�ʼ������
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void usart_init( void )
{
//    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
//    DCOCTL = CALDCO_1MHZ;

/* 1MHz����9600������
    P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSSEL_2;
    
    //UCA0BR0 = 106;                            // 1MHz 9600
    //UCA0BR1 = 0;                              // 1MHz 9600
    //UCA0BR0 = 0x03;                           // 8MHz 9600
    //UCA0BR1 = 0x00;                           // 8MHz 9600
    UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5    BRCLK/(UBR+(M7+...0)/8)
    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
*/

/* 32KHz����9600������*/
    P3SEL |= 0x30;                          // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSSEL_1;                   // CLK = ACLK
    UCA0BR0 = 0x03;                         // 32kHz/9600 = 3.41
    UCA0BR1 = 0x00;                         //
    UCA0MCTL = UCBRS1 + UCBRS0;             // Modulation UCBRSx = 3
    UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

//  IE2 |= UCA0RXIE;                        // Enable USCI_A0 RX interrupt
//  __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0, interrupts enabled
}

/*******************************************************************************
  * @brief  ���ڷ���һ���ֽ�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void UartPutchar( unsigned char c )
{
    while ( !( IFG2 & UCA0TXIFG ) );
    UCA0TXBUF = c;
    IFG2 &= ~UCA0RXIFG;
}

/*******************************************************************************
  * @brief  ���ڽ���һ���ֽ�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
unsigned char UartGetchar()
{
    unsigned char c;
    while ( !( IFG2 & UCA0RXIFG ) );
    c = UCA0RXBUF;
    IFG2 &= ~UCA0TXIFG;
    return c;
}

/*******************************************************************************
  * @brief  UART�����ж�
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
/*
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  while (!(IFG2&UCA0TXIFG));
  UCA0TXBUF = UCA0RXBUF;
}
*/

/*******************************************************************************
  * @brief  putchar,�����ض����Զ����Ǳ�׼�⺯��
  * @param  cΪ�����͵��ַ�
  * @retval None
  * @Note   �򴮿��ն˷���һ���ַ�
******************************************************************************
int putchar(int c)
{
    if(c == '\n')
    {
        while(UCA0STAT & UCBUSY);
        UCA0TXBUF = '\r';
    }
    while(UCA0STAT & UCBUSY);
    UCA0TXBUF = c;
    while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
    return c;
}
*/

/*******************************************************************************
  * @brief  ���ڴ�ӡ����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void print( char* str )
{
    while ( *str != '\0' ) // \0 ��ʾ�ַ���������־��ͨ������Ƿ��ַ���ĩβ
    {
        UartPutchar( *str );
        str++;
    }
}

/*******************************************************************************
  * @brief  ���ڴ�ӡ�س�����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void println( char* str )
{
    print( str );
    UartPutchar( 0x0a );
    UartPutchar( 0x0d );
}

/*******************************************************************************
  * @brief  ���ڴ�ӡ����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void printNumber( INT16U x )
{
    unsigned char a, b, c, d, e;
    a = x / 10000;
    x = x % 10000;
    b = x / 1000;
    x = x % 1000;
    c = x / 100;
    x = x % 100;
    d = x / 10;
    e = x % 10;
    UartPutchar( a + 0x30 );
    UartPutchar( b + 0x30 );
    UartPutchar( c + 0x30 );
    UartPutchar( d + 0x30 );
    UartPutchar( e + 0x30 );
    UartPutchar( 0x0d );
    UartPutchar( 0x0a );
}

/*******************************************************************************
  * @brief  �Ż����ڴ�ӡ����
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void printNumber1(unsigned long num, unsigned char base)
{
  
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
  unsigned long i = 0;
  long n;
  n=num;
  if (n == 0) {
    UartPutchar('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    UartPutchar((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
      
    UartPutchar( 0x0d );
    UartPutchar( 0x0a );
}



#endif



/**********************************FLASH**************************************/

#ifdef USE_FLASH

/*******************************************************************************
  * @brief  ��FLASH���ò���              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void FLASH_LOAD()
{
    INT8U i;
    INT8U *Flash_ptr=(INT8U *)STORAGE_START_ADDR;   //���ݶ�ȡָ��
    for(i=0;i<sizeof(struct _storage_data);i++)
    {
        Storage_union.Storage[i]=*(Flash_ptr+i);
    }
    if(Storage_union.StorageData.Soft_Version==DEF_SOFT_VERSION) //��֤�汾�ţ�flash�����Ƿ��������԰汾��Ϊ���ݵģ�
    {
        Report_time=Storage_union.StorageData.FLASH_REPORT_TIME;//�ϱ�ʱ�䣨ң�ⷢ�ͼ����
        ERROR_RSTTIME=Storage_union.StorageData.FLASH_ERROR_RSTTIME;          //���ϸ���ʱ��
        TF_MAX   = Storage_union.StorageData.FLASH_TF_MAX;                 //��·ͻ��ʱ��
        DELT_I   = Storage_union.StorageData.FLASH_DELT_I;                  //����ͻ��Խ����ֵ����·ͻ�������
        CHZ_TIME = Storage_union.StorageData.FLASH_CHZ_TIME;
        MAX_I    = Storage_union.StorageData.FLASH_MAX_I;                  //��������ֵ
        T1_MAX   = Storage_union.StorageData.FLASH_T1_MAX;                 //����30��ÿ30msһ�Σ�
        T2_MAX   = Storage_union.StorageData.FLASH_T2_MAX;
        T3_MAX   = Storage_union.StorageData.FLASH_T3_MAX;
        T4_MAX   = Storage_union.StorageData.FLASH_T4_MAX;
        VBAT_MIN = Storage_union.StorageData.FLASH_VBAT_MIN;             //��ص�ѹ����ֵ
        for(i=0;i<14;i++)
        {
            SampleTable[i]=Storage_union.StorageData.FLASH_SAMPLE_TABLE[i];
        }
        
    }

}

/*******************************************************************************
  * @brief  ���ݱ���              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void FLASH_SAVE()
{
    INT16U i;
    INT8U *Flash_ptr=(INT8U *)STORAGE_START_ADDR;

#ifdef USE_WATCHDOG   
    WDTCTL = WDTPW + WDTHOLD;
#endif
    _DINT();
    
    Flash_Clr((INT16S *)0x1000);           //����D��
    FCTL3=FWKEY;
    FCTL1=FWKEY+WRT;                       //Set WRT bit for write operation
    
   
    Storage_union.StorageData.Soft_Version        =    DEF_SOFT_VERSION;
    Storage_union.StorageData.FLASH_REPORT_TIME   =    Report_time ;         //�ϱ�ʱ�䣨ң�ⷢ�ͼ����
    Storage_union.StorageData.FLASH_ERROR_RSTTIME =    ERROR_RSTTIME ;     //���ϸ���ʱ��
    Storage_union.StorageData.FLASH_TF_MAX        =    TF_MAX    ;          //��·ͻ��ʱ��
    Storage_union.StorageData.FLASH_DELT_I        =    DELT_I ;              //����ͻ��Խ����ֵ����·ͻ�������
    Storage_union.StorageData.FLASH_CHZ_TIME      =    CHZ_TIME  ;
    Storage_union.StorageData.FLASH_MAX_I         =    MAX_I ;              //��������ֵ
    Storage_union.StorageData.FLASH_T1_MAX        =    T1_MAX ;             //����30��ÿ30msһ�Σ�
    Storage_union.StorageData.FLASH_T2_MAX        =    T2_MAX;        
    Storage_union.StorageData.FLASH_T3_MAX        =    T3_MAX ;       
    Storage_union.StorageData.FLASH_T4_MAX        =    T4_MAX   ;     
    Storage_union.StorageData.FLASH_VBAT_MIN      =    VBAT_MIN ;         //��ص�ѹ����ֵ
    for(i=0;i<14;i++)
    {
        Storage_union.StorageData.FLASH_SAMPLE_TABLE[i] = SampleTable[i];
    }
    for(i=0;i<sizeof(struct _storage_data);i++)
    {
       *(Flash_ptr+i) = Storage_union.Storage[i];
    }

    //*Flash_ptr++=0x22;                                                         
    //(INT16U *)(&FLASH_REPORT_TIME)=Report_time;                                 
/* ����д��                                                                       
    FLASH_REPORT_TIME = Report_time;     //�ϱ�ʱ�䣨ң�ⷢ�ͼ����             
    FLASH_ERROR_RSTTIME = ERROR_RSTTIME; //���ϸ���ʱ��                           
    FLASH_TF_MAX = TF_MAX;          //��·ͻ��ʱ��
    FLASH_DELT_I = DELT_I;             //����ͻ��Խ����ֵ����·ͻ�������

    FLASH_MAX_I = MAX_I;               //��������ֵ
    FLASH_T1_MAX= T1_MAX;               //����30��ÿ30msһ�Σ�
    FLASH_T2_MAX= T2_MAX;
    FLASH_T3_MAX= T3_MAX;
    FLASH_T4_MAX= T4_MAX;
    FLASH_VBAT_MIN= VBAT_MIN; //��ص�ѹ����ֵ
 */  
    FCTL1 = FWKEY;                           // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                    // Set LOCK bit
    
#ifdef USE_WATCHDOG
    WDTCTL = WDT_ARST_1000;
#endif
    _EINT();
    
}


/*******************************************************************************
  * @brief  FLASH  ��������              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void Flash_Clr(INT16S *Data_ptr)
{
    FCTL2 = FWKEY+FSSEL_1+FN4+FN2+FN1+FN0;       // MCLK/24 for Flash Timing Generator
    FCTL3=FWKEY;
    FCTL1 =FWKEY+ERASE;                   //�����β���
    *Data_ptr=0;                          //dummy  write
    
}

#endif


/*******************************************************************************
  * @brief  �о�״̬��
  * @param  None
  * @retval None
  * @Note   10ms���һ�β�������һ��
*******************************************************************************/
void statuscheck( INT16U value )
{
    //INT16U i;
    INT16U temp;
/* �������ת��  */ 
    if(SampleDelay==0 & (P2IN&0x20)==0)//����(��Ϊ����)
    {
        //if(flag.SampleMode!=0)       //��������жϻ������⣬���Ҳ���ԭ��
         {
#ifdef NRF905
			if(status!=0)
			{
				nRF905_Report(0x32,0,0);
			}
#endif
            status=0; 
			flag.PowerDown=1;  
         }
    }
    // if(value<10)            //10ms����1��
    // {
           // Set_ADMode(10);             
    
    // }
    else if(value<55)
    {
        //if(flag.SampleMode!=5)
        { 
            TA1CTL  = TASSEL_1 + MC_1;        //ACLKʱ��
            flag.SampleMode=5;
            TA1CCR0 = 164;
        }  
    }
    else
    {
        //if(flag.SampleMode!=1)
        {
            TA1CTL  = TASSEL_2 + MC_1 + ID_3;        //SMCLKʱ��
            flag.SampleMode=1;
            TA1CCR0 = 1000;
        }  
    }
	
	pre_i = last_i;
    last_i = cur_i;
    cur_i = value;

#if 0    //��̬STEADY_I
	if(cur_i<150)
	{
		STEADY_I=5;
	}
	else if(cur_i<250)
	{
		STEADY_I=10;
	}
	else if(cur_i<350)
	{
		STEADY_I=15;
	}
	else if(cur_i<550)
	{
		STEADY_I=20;
	}
	else 
	{
		STEADY_I=30;
	}
#endif


#ifdef USART
    //printNumber1((long)cur_i,10);
#endif	
    
    
    //return;
    
//�������о�
    if ( cur_i > MAX_I && flag.Error!=3)       //�����ɱ���
    {
        Tm++;
        if(Tm>930)       //��ʱ10s��1000/1.075f��
        {
            Tm=0;
            fanhong();             //����
			flag.Error = 3;
#ifdef NRF905
            nRF905_Report(0x20,(INT16U)If,0);
#endif
            status = 0;            //�������
            led_flash(100,4);
        }

    }
    else
    {
         Tm=0;
    }

 
//�ж϶�·
    switch ( status )
    {
    case 0:                         //��ʼ״̬(�������������λ)
        if ( cur_i >= 1 )       //����ֵ������С������������
        {
            In_t++;    //ʱ�����
        }
        else
        {
            In_t = 0;    //����Ϊ����λ(��ֹ�ǹ������غ�ӿ����)
        }
        if ( In_t > 930 )          //��������10�루1000/1.075f��
        {
            In_t = 0;               //����
            if(flag.Error == 0)     //����״̬�ϵ��뱣֤����
            {
                fanbai();
#ifdef NRF905
                nRF905_Report(0x31,0,0);
#endif
                
            }
            else if ( flag.Error == 1)     //�Ƿ��������Զ�·����
            {
                RstTime_Count=0;
                fanbai();           //���飨���ƣ�
				flag.Error = 0;        //��������
#ifdef NRF905
                nRF905_Report(0x17,0,0);
#endif

            }
            else
            {
#ifdef NRF905
            nRF905_Report(0x31,0,0);
#endif
            }
            f_steady = 0;           //�ȶ���־λ����
            normal_i = cur_i;       //��ȡ�������е���
            if(normal_i<(2*DELT_I))JUGE_DELT_I=DELT_I;
                    else JUGE_DELT_I=(normal_i/2);
           
            led_flash(100, 2 );     //����
#ifdef USART	
	//println("go case 1");
#endif	
            status = 1;             //������������״̬
        }
        break;
    case 1:                         //��������
//ʧ�����
#if 0
        if(cur_i < DEF_In)               //����
        {
            T_zero++;
            if(T_zero>300)          //3S����
            {
              T_zero=0;
#ifdef NRF905
              nRF905_Report(0x32,0,0);
#endif
              status=0;  
              break;
            }     
        }
        else
        {
            T_zero=0;
        }
#endif
//����ѹ����  
/**/  
        //temp=(INT16U)(last_i*0.03);
        temp=last_i>>5;
        if(temp<3)temp=3;
        if ( abs(cur_i,last_i) < temp) //2�ε�����С��10��Ϊ�ȶ�
        {
            delt_t=0;
            f_steady++;             //�����ȶ���־λ
            if ( f_steady > 2 )     //3�ζ��ȶ���Ϊƽֱ��
            {
                normal_i=last_i;
                
                //if(normal_i<(2*DELT_I))JUGE_DELT_I=DELT_I;
                if(normal_i<(DELT_I<<1))JUGE_DELT_I=DELT_I;
                    //else JUGE_DELT_I=(normal_i/2);
                    else JUGE_DELT_I=(normal_i>>1);
            }
        }
        else
        {
            delt_t++;
            if ( RiseEdge(cur_i,normal_i) > (INT16S)JUGE_DELT_I ) //�����仯��������ͻ��Խ����ֵ
            {
#ifdef USART	
	//println("go case 2");
#endif	
                    Tf = 0;
                    If=cur_i;
                    status = 2;
                    Ts=0;

            }
        }
  
        
/*
        if ( absf( cur_i - last_i ) < STEADY_I ) //2�ε�����С��10��Ϊ�ȶ�
        {
            f_steady++;             //�����ȶ���־λ
            if ( f_steady > 2 )     //3�ζ��ȶ���Ϊƽֱ��
            {
                f_steady = 0;       //�ȶ�״̬����
                if ( absf( cur_i - normal_i ) > DELT_I ) //�ȶ������仯ֵ��������ͻ��Խ����ֵ
                {
                    Tf = 3;                  //���ִ���10ms
                    status = 2;              //����ߵ���״̬
                }
                else
                {
                    normal_i=cur_i;
                }

            }
        }

        else                             //�����仯�ϴ�
        {
            f_steady = 0;
            if ( ( cur_i - normal_i ) > DELT_I ) //�����仯��������ͻ��Խ����ֵ�������ֶ��ӵ磩
            {
                Tf = 0;
                status = 2;
            }
            else
            {
                delt_t++;                //�����仯ʱ��+1
            }
        }
*/
        break;
    case 2:                            //ͻ�䣨�ߵ���״̬��
        //uart_normal_i=1;
        if ( abs(cur_i,last_i ) < STEADY_I ) //2�ε�����С��10��Ϊ�ȶ�
        {
            f_steady++;                //�����ȶ���־λ��
            if ( f_steady > 2 )        //3�ζ��ȶ���Ϊƽֱ��
            {
                f_steady = 0;          //�ȶ�״̬����
                if ( RiseEdge(cur_i,normal_i ) > (INT16S)JUGE_DELT_I ) //�ȶ������仯ֵ����100
                //if ( abs(cur_i,normal_i ) > DELT_I )
                {
                    //Tf += 3;
                    if(cur_i>If)If=cur_i;
                }
                else if ( cur_i >= DEF_In )
                {
                    status = 1;        //���ɲ�������
                }
                else if ( cur_i < DEF_In ) //����Ϊ��
                {
#ifdef USART
                  //      printNumber1((long)Tf,10);
#endif
                    Tf=(INT16U)(1.075f*Tf);
                   // if ( Tf > 0 && Tf < TF_MAX  && normal_i>5 )
                    if ( Tf < TF_MAX  && normal_i>=DEF_In )
                    {

                        //Ts=0;
                        Tf=0;
                        status=3;
                    }
                    else
                    {
                        Tf=0;
                        status=1;
                    }
                }
            }
        }
        else
        {
            f_steady = 0;
        }
        
        if (abs(cur_i,normal_i) > (JUGE_DELT_I-10) ) //��Ȼ�ڸߵ�����Ҳ���ֶ����䣩
      //if (abs(cur_i,normal_i) > DELT_I ) //��Ȼ�ڸߵ�����Ҳ���ֶ����䣩
        {
            if(cur_i>If)If=cur_i;
            Tf++;
#if 1 //���������BUG����
			if (Tf> 500)
			{
				Tf=0;
				status=1;
			}
#endif			
			
			
			
        }
        
        if(cur_i<DEF_In)
        {
            Ts++;
        }
        else
        {
            Ts=0;
        }
        break;
        
    case 3:
        if(cur_i < DEF_In)
        {
            Ts++;
            if(Ts>CHZ_TIME)        //���غ�բ
            {
                Ts=0;
                fanhong();           //����
				flag.Error = 1;
#ifdef NRF905
                nRF905_Report(0x19,(INT16U)If,0);
                If=0;
#endif
                status = 0;
                led_flash(100, 4 );
            }
        }
        else if(cur_i>=DEF_In)     
        {
           Tc=0;
           status=4;
        }
        break;
    case 4:
        if(cur_i>=DEF_In)
        {
            Tc++;
            if(Tc>300)
            {
                fanhong();           //����
				flag.Error = 4;
#ifdef NRF905
                nRF905_Report(0x16,(INT16U)If,0);
                If=0;
#endif
                
                status = 0;
                led_flash(100, 4 );

            }
                
            /*
            if ( absf( cur_i - last_i ) < STEADY_I ) //2�ε�����С��10��Ϊ�ȶ�
            {
                f_steady++;             //�����ȶ���־λ
                if ( f_steady > 2 )     //3�ζ��ȶ���Ϊƽֱ��
                {
                    normal_i=cur_i;
                    //Tc+=3;
                }
            }
            */
        }
        else if(cur_i<DEF_In)         //���㣨���ù��ϣ�
        {
            fanhong();           //����
			flag.Error = 1;
#ifdef NRF905
            nRF905_Report(0x19,(INT16U)If,0);
            If=0;
#endif
            status = 0;
            led_flash(100, 4 );
        }

        break;
       
    }

//�жϽӵ�

switch ( flag.jiedi )       //�ӵ��о�
{
    case 0:
        if ( abs(cur_i,last_i) < STEADY_I ) //2�ε�����С��10��Ϊ�ȶ�
        {
            Groundsteady++;             //�����ȶ���־λ
            if ( Groundsteady > 2 )     //3�ζ��ȶ���Ϊƽֱ��
            {
                Groundsteady=0;
				flag.jiedi_min_ok=1;
                maichong_i=cur_i;
				if(maichong_i<150)
				{
					PUL_MIN_I=5;
				}
				else if(maichong_i<250)
				{
					PUL_MIN_I=15;
				}
				else if(maichong_i<350)
				{
					PUL_MIN_I=20;
				}
				else if(maichong_i<550)
				{
					PUL_MIN_I=25;
				}
				else
				{
					PUL_MIN_I=30;
				}
            }
        }

#ifdef USART	
           // UartPutchar('0');
#endif
		if(flag.jiedi_min_ok==1)
		{
			if(RiseEdge(cur_i,maichong_i)>PUL_MIN_I & cur_i>last_i)
			{
				flag.jiedi_max_ok=0;
				TIMCNT_Ptr = &T1;
				flag.TIMCNT_EN=1;
				flag.jiedi=1;
			}
		}
        break;
    case 1:
		if(flag.jiedi_max_ok==0)
		{
			if ( abs(cur_i,last_i) < STEADY_I ) //2�ε�����С��10��Ϊ�ȶ�
			{
				Groundsteady++;             //�����ȶ���־λ
				if ( Groundsteady > 2 )     //3�ζ��ȶ���Ϊƽֱ��
				{
					Groundsteady=0;
					flag.jiedi_max_ok=1;
					Ig=cur_i;
					if(Ig<150)
					{
						PUL_MAX_I=5;
					}
					else if(Ig<250)
					{
						PUL_MAX_I=15;
					}
					else if(Ig<350)
					{
						PUL_MAX_I=20;
					}
					else if(Ig<550)
					{
						PUL_MAX_I=25;
					}
					else
					{
						PUL_MAX_I=30;
					}
				}
			}
		}
		else
		{
			 if(RiseEdge(Ig,cur_i)>(INT16S)PUL_MAX_I & cur_i<last_i)
			 {
				flag.TIMCNT_EN=0;
				T1--;
#ifdef USART
				print("T1=");
				printNumber1((long)T1,10);
#endif
				if(T1>(T1_MAX-5) & T1 <(T1_MAX+5))
				//if(T1>0)
				{
					T1=0;
					TIMCNT_Ptr = &T2;
					flag.TIMCNT_EN=1;
					flag.jiedi=2;  
				}
				else
				{
					T1=0;
					num=0;
					flag.jiedi=0;
				}
			 }
			 else
			 {
				//T1++;
	#ifdef USART	
				//UartPutchar('1');
	#endif
			 }
		}
		
		if(T1>200)
		{
			flag.TIMCNT_EN=0;
			T1=0;
			num=0;
			flag.jiedi=0;
		}
         break;
    case 2:
         if(RiseEdge(cur_i,maichong_i)>(INT16S)PUL_MIN_I & cur_i>last_i)
         {
			flag.TIMCNT_EN=0;
                        T2++;
#ifdef USART
			print("T2=");
			printNumber1((long)T2,10);
#endif
            if(T2>(T2_MAX-5) & T2<(T2_MAX+5))  
            //if(T2>0)      
            {
                T2=0;
				TIMCNT_Ptr = &T3;
				flag.TIMCNT_EN=1;
                flag.jiedi=3;  
            }
            /**/
            else if(T2>(T4_MAX-5) & T2<(T4_MAX+5))
            {
                T2=0;
                num++;
				TIMCNT_Ptr = &T1;
				flag.TIMCNT_EN=1;
                flag.jiedi=1;
            }
            else
            {
                T2=0;
                num=0;
                flag.jiedi=0;
            }
         }
         else
         {
           //T2++;
#ifdef USART	
           //UartPutchar('2');
#endif    
           if(T2>200)
            {
				flag.TIMCNT_EN=0;
                T2=0;
                num=0;
                flag.jiedi=0;
            }
         }
         break;
     case 3:
         if(RiseEdge(Ig,cur_i)>(INT16S)PUL_MAX_I & cur_i<last_i)
         {
             flag.TIMCNT_EN=0;
#ifdef USART
			 print("T3=");
			 printNumber1((long)T3,10);
#endif
             T3--;
             if(T3>(T3_MAX-5) & T3<(T3_MAX+5))
             //if(T3>0)     
             {
                T3=0;
                num++;
				TIMCNT_Ptr = &T4;
				flag.TIMCNT_EN=1;
                flag.jiedi=4; 
             }
            else
            {
                T3=0;
                num=0;
                flag.jiedi=0;
            }
           
         }
         else
         {
           //T3++;
#ifdef USART	
          // UartPutchar('3');
#endif
           if(T3>200)
           {
				flag.TIMCNT_EN=0;
                T3=0;
                num=0;
                flag.jiedi=0;
           }
         }
         break;   
     case 4:
           if(num==3 & T4>(T4_MAX-5))
           {
			   flag.TIMCNT_EN=0;
                T4=0;
                flag.jiedi = 0;
                num=0;
                fanhong();             //����
				flag.Error = 2;
#ifdef NRF905
                nRF905_Report(0x18,Ig-maichong_i,0);
#endif
                
                //status = 0;            //�������
                led_flash(100,4 ); 
           }  
                
         if(RiseEdge(cur_i,maichong_i)>(INT16S)PUL_MIN_I & cur_i>last_i)
         {
			 flag.TIMCNT_EN=0;
#ifdef USART
			print("T4=");
			printNumber1((long)T4,10);
#endif
             T4++;
             if(T4>(T4_MAX-5) & T4<(T4_MAX+5))
             //if(T4>0)  
             {          
/* T1 T2 T3 T4 �������      
           print("T1=");
           printNumber(T1);
           T1=0;
            
           print("T2=");
           printNumber(T2);
           T2=0;
            
           print("T3=");
           printNumber(T3);
           T3=0;
            
           print("T4=");
           printNumber(T4);
           T4=0;
 */   
                T4=0;
                //printNumber1(num,10);
				TIMCNT_Ptr = &T1;
				flag.TIMCNT_EN=1;
                flag.jiedi=1; 
             }
            else
            {
                T4=0;
                num=0;
                flag.jiedi=0;
            }

         }
         else
         {
           //T4++;
#ifdef USART	
          // UartPutchar('4');
#endif

           if(T4>200)
            {
				flag.TIMCNT_EN=0;
                T4=0;
                num=0;
                flag.jiedi=0;
            }

         }
         break;       
}

}


/*********************************END OF FILE**********************************/
