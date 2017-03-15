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
union     //flash数据存储联合体
{                              
  unsigned char Storage[sizeof(struct _storage_data)];          
  struct _storage_data StorageData;               
}Storage_union;

/* 指定地址 
const INT32U FLASH_REPORT_TIME @ 0x1000  = DEF_REPORT_TIME * 60000; //上报时间（遥测发送间隔）
const INT32U FLASH_ERROR_RSTTIME @ 0x1004  = DEF_ERROR_RSTTIME * 3600000; //故障复归时间
const INT16U FLASH_TF_MAX @ 0x1008 = DEF_TF_MAX/10;          //短路突变时间
const INT16U FLASH_DELT_I @ 0x100A = DEF_DELT_I;             //电流突变越线阈值（短路突变电流）

const INT16U FLASH_MAX_I @ 0x100C = DEF_MAX_I;               //过负荷阈值
const INT16U FLASH_T1_MAX @ 0x100E= DEF_T1_MAX/10;               //除以10（每10ms一次）
const INT16U FLASH_T2_MAX @ 0x1010= DEF_T2_MAX/10;
const INT16U FLASH_T3_MAX @ 0x1012= DEF_T3_MAX/10;
const INT16U FLASH_T4_MAX @ 0x1014= DEF_T4_MAX/10;
const INT16U FLASH_VBAT_MIN @ 0x1016= DEF_VBAT_MIN; //电池电压低阈值
*/ 
/* 可设参数 */
INT32U Report_time = DEF_REPORT_TIME * (INT32U)1000; //上报时间（遥测发送间隔）
INT32U ERROR_RSTTIME = DEF_ERROR_RSTTIME * 60000; //故障复归时间(单位分钟)
INT16U TF_MAX = DEF_TF_MAX/10;          //短路突变时间
INT16U DELT_I = DEF_DELT_I;             //电流突变越线阈值（短路突变电流）
INT16U CHZ_TIME=DEF_CHZ_TIME/10;        //重合闸时间
INT16U MAX_I = DEF_MAX_I;               //过负荷阈值
INT16U T1_MAX = DEF_T1_MAX/10;          //接地脉冲1时间（除以10（每10ms一次））
INT16U T2_MAX = DEF_T2_MAX/10;          //接地间隔1时间
INT16U T3_MAX = DEF_T3_MAX/10;          //接地脉冲2时间
INT16U T4_MAX = DEF_T4_MAX/10;          //接地间隔2时间
INT16U VBAT_MIN = DEF_VBAT_MIN;         //电池电压低阈值


//{0xfc,     0xf5,       0xe2,       0xef,       0xef,       0xf0,       0xf4,       0xfa,     0x03,       0x0a,     0x22    ,   0x05};
//--------------------0~50--50~80--80~100--100~150--150~200--200~250--250~300--300~350--350~400--400~500--500~600--600~630

/* 内部参数 */
INT16U STEADY_I = DEF_STEADY_I;         //稳定阈值
INT16U last_i, cur_i, normal_i, If;     //当前电流和上次电流
INT16U pre_i;
unsigned char status;                   //状态标志
long  In_t;                             //启动计时

INT16U delt_t, Tf, Ts,Tc;               //状态数据

INT32U ms_count;                        //时间计数

INT32U RstTime_Count = 0;               //定时复归计时
INT16U Error_FlashTime=0;               //有故障时闪灯时间计数
unsigned char f_steady;                 //稳定标志

//接地数据
INT16U PUL_MIN_I = DEF_PUL_MIN_I;       //最小接地脉冲
INT16U PUL_MAX_I = DEF_PUL_MAX_I;       //最大接地脉冲

INT16U maichong_i;                      //脉冲参考基准值
INT16U Groundsteady;                    //接地稳定阈值

INT16U LED_TimeCount;                   //闪灯延时计数

INT16U T1, T2, T3, T4;                  //接地脉冲时间计数
unsigned char num;                      //接地脉冲计数

INT16U Id,Ig;                           //接地故障电流

INT16U T_zero;                          //失电时间计数
INT16U Tm;                              //过阈值时间计数             
INT16U CPU_TEMP;                        //CPU温度值         
INT16U VBAT;                            //电池电压值

INT32U SampleBuffer[10];                //采样缓冲区

unsigned char SampleCount = 0;          //采样值指针
INT32U SampleValue;                     //采样值
INT32U oldSampleValue;                  //上次采样值


INT16U LED_DELAY=DEF_LED_DELAY;         //LED闪烁延时
INT16U SEND_DELAY;                      //905发送延时
INT16U PWRDN_DELAY;                     //系统休眠延时
INT32U RX_DELAY;                        //905休眠延时
INT32U PRE_RX_DELAY=DEF_RX_DELAY;       //905休眠延时时间预设值（在每次设为发送模式后会重新装载此值作为延时时间）

INT32U SingleSample_count;              //单次采样（电池电压和CPU温度采样）延时计数
INT32U Report_time_count;               //定时上报延时计数

INT16U SampleTemp;                      //单次采样临时存放变量
INT16U jiedi_delay;                     //防止在脉冲过程中改变参考电流值

flagbit flag;                           //整个系统标志位汇总

INT16U SampleDelay=DEF_SAMPALE_DELAY;   //采样延时

INT16U JUGE_DELT_I;                     //作为判据的电流突变阈值

INT16U SOFT_WDG=65535;

extern char TxBuf[TxRxBuf_Len];
extern INT16U SampleTable[14];


INT16U * TIMCNT_Ptr;
//INT16U tam[10];
/* Functions -----------------------------------------------------------------*/
#ifdef USE_EXIT
/*******************************************************************************
  * @brief  外部中断初始化
  * @param  None
  * @retval None
  * @Note   P2.5端口接比较器输入端，
            电流值在5~70，比较器会输出一个脉宽随电流值变化的方波
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
//P2口外部中断服务函数
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
  * @brief  内部比较器初始化
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void compare_init()
{
    CACTL1 |= CAREF_1 + CAON + CAREF_2  /*+ CAIE */;          // 0.25 Vcc On -comp, (参考电压加在比较器负极)
    //CACTL2 |= CAF;
    CACTL2 = P2CA1 + P2CA3;              //使用CA5

}

/*******************************************************************************
  * @brief  比较器中断
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
  * @brief  引脚设置
  * @param  None
  * @retval None
  * @Note   配置未使用引脚防止浮空电流（实测200uA）
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
  * @brief  浮点型取绝对值
  * @param  None
  * @retval None
  * @Note   程序代码量大，可以优化
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
  * @brief  INT16U取绝对值
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
  * @brief  判断上升沿
  * @param  None
  * @retval None
  * @Note   编译器对带符号数支持有问题
*******************************************************************************/
INT16S RiseEdge( INT16U a,INT16U b )
{
    INT16S x = (INT16S)a;
    INT16S y = (INT16S)b;  
    INT16S num = x-y;

    return num;
}

/*******************************************************************************
  * @brief  LED初始化
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
  * @brief  LED闪烁 1s延时
  * @param  n：LED闪烁次数
            time:延时时间，单位ms
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
  * @brief  翻牌初始化
  * @param  None
  * @retval None
  * @Note   设置P1口1、2、3、6、7引脚为输出端口
*******************************************************************************/
void flop_init( void )
{
    P1DIR |= BIT1 + BIT2 + BIT3 + BIT6 + BIT7;
}

/*******************************************************************************
  * @brief  翻红
  * @param  None
  * @retval None
  * @Note   倍压电路
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
  * @brief  翻白
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




/************************************定时器************************************/



#ifdef USE_TIMER

#ifdef USE_TIMERA0
/*******************************************************************************
  * @brief  定时器A0初始化
  * @param  None
  * @retval None
  * @Note   时钟选择ACLK，增计数模式，清计数值
*******************************************************************************/
void TimerA0_Init( void )
{
    TA0CCR0 = 327;                    //定时10ms
    //TA0CCR0 = 164;                  //定时5ms
    //TA0CCR0 = 16384;
    //TA0CCR0 = 32768;
    TA0CTL  = TASSEL_1 + MC_1;        //ACLK时钟
}

/*******************************************************************************
  * @brief  增计数模式下，开始计时
  * @param  None
  * @retval None
  * @Note   允许中断
*******************************************************************************/
void Start_Timer_A0( void )
{
    TA0CTL |= TACLR;
    TA0CCTL0 |= CCIE ;   //开启定时器
    _EINT();
}

/*******************************************************************************
  * @brief  增计数模式下，停止计时
  * @param  None
  * @retval None
  * @Note   关闭中断
*******************************************************************************/
void Stop_Timer_A0( void )
{
    TA0CCTL0 &= ~CCIE ;               //停止模式
}

/*******************************************************************************
  * @brief  定时器A0中断
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 ( void )
{
   // int i;
    /* 重置看门狗 */
#ifdef USE_WATCHDOG
    WDTCTL = WDTPW + WDTCNTCL;
#endif
/* 时间计数 */
    ms_count+=10;
    if(flag.Error>0)        //定时复归
    {
        RstTime_Count+=10;
		Error_FlashTime++;
        if(Error_FlashTime>300)//1s一闪（取模运算会有大量功耗实测10uA）
        {
            led_flash(100,1);
        }
        if(RstTime_Count>ERROR_RSTTIME)
        {
            RstTime_Count=0;
            fanbai();              //复归（翻牌）
            nRF905_Report(0x17,0,0);
            flag.Error = 0;        //错误清零
        }
        
    }

    if ( ms_count >= 86400000 ) //当时间达到1天
    {
        ms_count = 0;
    }


    
    
    
/* LED延时闪烁 */ 
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

    
 
/* 905接收保持  */
    if(RX_DELAY>0)
    {
        RX_DELAY-=10;
    }    
    else if(flag.nrf905_mode==3)
    {
        TRX_CE_0;
        //PWR_0;
        flag.nrf905_mode=1; //休眠模式
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
            PRE_RX_DELAY=DEF_RX_DELAY;//重设休眠延时预设值
            flag.nrf905_mode=0;//掉电模式
        }
        
    }

 
/* 报文发送延时  */
    if(SEND_DELAY>0)
    {
        SEND_DELAY-=10;

    }

 
/* 1s定时电压采样 */
    SingleSample_count+=10;
    if ( SingleSample_count >= 60000 ) //10S中断
    {
        SingleSample_count=0;
        flag.RequireTemp=1; //CPU温度采样
        if(flag.VBAT_Low==0)
        {
            flag.RequireVBAT=1; //电池电压采样
        }
    }
/* 取模 电流消耗大
    if ( ms_count % 60000 == 0 ) //10S中断
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
         if((P2IN&0x20)==0)SampleDelay-=10;//采样休眠，即系统休眠（在比较器有输出时一直重设此值，一旦电流<5A比较器停止输出，此值会降为0，系统进入休眠）
    }



/* 定时上报 */
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
	else //软件复位
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
  * @brief  定时器A1初始化
  * @param  None
  * @retval None
  * @Note   时钟选择ACLK/SMCLK，增计数模式，清计数值
*******************************************************************************/
void TimerA1_Init( void )
{
/* 32KHz晶振 */
    //TA1CCR0 = 33;                   //定时1ms
    TA1CCR0 = 164;                  //定时5ms
    //TA1CCR0 = 16384;
    //TA1CCR0 = 327;                    //定时10ms
    //TA1CCR0 = 32768;                  //定时1s
    TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟

/* 8M晶振 
    TA1CCR0 = 5000;
    TA1CTL  = TASSEL_2 + MC_1 + ID_3; //选择SMCLK 8分频
*/
}

/*******************************************************************************
  * @brief  增计数模式下，开始计时
  * @param  None
  * @retval None
  * @Note   允许中断
*******************************************************************************/
void Start_Timer_A1( void )
{
    //TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟
    TA1R=0;
    TA1CTL |= TACLR;
    TA1CCTL0 |= CCIE ;   //开启定时器
    _EINT();
}

/*******************************************************************************
  * @brief  增计数模式下，停止计时
  * @param  None
  * @retval None
  * @Note   关闭中断
*******************************************************************************/
void Stop_Timer_A1( void )
{
    TA1CCTL0 &= ~CCIE ;               //停止模式
     //TA1CTL  = TASSEL_1 + MC_0;        //ACLK时钟
}

/*******************************************************************************
  * @brief  定时器A1中断
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A1 ( void )
{
    /* 唤醒AD采样 */
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
  * @brief  ADC初始化
  * @param  None
  * @retval None
  * @Note   内部2.5V参考电压
            自带5M时钟
*******************************************************************************/
void adc_init()
{
    ADC10CTL0 &= ~ENC;                                       //在改变设置前停止A/D转换
    while (ADC10CTL1 & BUSY);                                //等待ADC10核心激活
    ADC10CTL0 = SREF_1 +  REF2_5V /*+ REFON */+ ADC10SHT_3 + ADC10ON + ADC10IE +/* REFOUT */+ ADC10SR /*+ REFBURST */; // ADC10ON, interrupt enabled
    ADC10AE0 |= 1;                         // P2.0 ADC option select
    //ADC10CTL1 = CONSEQ_2;//SHS_1;
    //ADC10CTL0 |= ENC;
    //ADC10CTL0 |= 0x1<<13;       //选用内部参考电压   SREF_1
    //ADC10CTL0 |= 0x1<<5;        //开内部参考电压
    //ADC10CTL0 |= 0x1<<6;        //选用2.5V内部参考电压  REF2_5V
}

/*******************************************************************************
  * @brief  ADC关闭
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
  * @brief  单通道单次采样（有中断）
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
  * @brief  唤醒AD
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void wakeAD()
{
    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );
//  ADC10AE0 |= 3;                         // P2.0 ADC option select

    ADC10CTL0 |= REFON;                     //开启内部基准电压
    
    ADC10CTL1 = INCH_0;

    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
}

/*******************************************************************************
  * @brief  单次采样不用中断
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
INT16U SingleSample(unsigned char channel)
{
    INT16U temp;
    INT16U ctl1;                 //保存ADC当前的寄存器数据
    INT16U ctl0;                 //保存ADC当前的寄存器数据

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
    ADC10CTL1 = ctl1;                         //关闭并复位ADC状态
    ADC10CTL0 = ctl0;                         //关闭并复位ADC状态
    return temp;
}

/*******************************************************************************
  * @brief  数组冒泡算法，排序
  * @param  unsigned int *pData：把需要排序数据指针：unsigned char Count：数据个数
  * @retval None
  * @Note   数组值从小到大排序
******************************************************************************
void bubble(INT16U *a,int n) //定义两个参数：数组首地址与数组大小
{
    int i,j,temp;
    for(i=0;i<n-1;i++)
        for(j=i+1;j<n;j++) //注意循环的上下限
            if(a[i]>a[j])
            {
                temp=a[i];
                a[i]=a[j];
                a[j]=temp;
            }
}
*/

/*******************************************************************************
  * @brief  ADC10中断服务函数
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR( void )
{
    
    int i = 0;
    //SampleValue =ADC10MEM;
    /* 如果是电池电压检测采样中断 
    if ( flag.VBATSample == 1 )
    {
        flag.VBATSample=0;
        SampleTemp=ADC10MEM;
        __bic_SR_register_on_exit( CPUOFF );      // Clear CPUOFF bit from 0(SR)

    }
    */
  //else
  //{
         /* 采样值直接输出
        printNumber((long)ADC10MEM);
          */
  //}
          
          
          
    /* 均方根值 
    //double temp;
    //temp = ( double )ADC10MEM;
    
    
    
    
    //SampleBuffer[SampleCount++] = pow(ADC10MEM, 2 );
    SampleBuffer[SampleCount++] =ADC10MEM;
    

    //SampleBuffer[SampleCount++] = temp*temp;
    if ( SampleCount > 9 )
    {
        //SampleValue = 0;
        SampleCount = 0;
        // 放入主程序中 
        for ( i = 0; i < 10; i++ )
        {
            SampleValue += SampleBuffer[i];
        }

        SampleValue =sqrt( SampleValue / 10 ); //取均方差
        //oldSampleValue=SampleValue;
        //一阶滞后滤波
        oldSampleValue = ( 0.5 * oldSampleValue ) + ( 0.5 * SampleValue );
       
        flag.SampleReady = 1;
        LPM3_EXIT;
        //flag.RequireVBAT=1;
    }
*/

    /* 递推平均滤波

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

    /* 10次取平均 */
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
            //SampleValue=sqrt(SampleValue/10);  //取均方差
           // SampleValue1 /= 10;                   //取平均值
            //一阶滞后滤波
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
    switch(flag.SampleMode)           //根据不同的采样模式设置不同的中断服务
    {
        case 0:                       //5ms采样1次
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
        case 1:                       //1ms采样1次
            SampleBuffer[SampleCount++] = (INT32U)pow(ADC10MEM, 2 );
            if ( SampleCount > 9 )
            {
                SampleValue = 0;
                SampleCount = 0;
                // 放入主程序中 
                for ( i = 0; i < 10; i++ )
                {
                    SampleValue += SampleBuffer[i];
                }

                SampleValue =(INT32U)sqrt( (double)(SampleValue / 10 )); //取均方差
                //oldSampleValue=SampleValue;
                //一阶滞后滤波
                //oldSampleValue1 = (INT32U)(( 0.5 * oldSampleValue ) + ( 0.5 * SampleValue ));
                oldSampleValue=SampleValue;
                flag.SampleReady = 1;
                //LPM3_EXIT;
                LPM0_EXIT;
                //flag.RequireVBAT=1;
             }
            break;
        case 5:                        //5ms采样1次
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
        case 10:                       //10ms采样1次
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
//关断内部稳压器(稳压器一直开启功耗很大，实测200uA)
    ADC10CTL0 &= ~ENC;
    while ( ADC10CTL1 & BUSY );
    ADC10CTL0 &= ~REFON;


}

/*******************************************************************************
  * @brief  设置AD采样模式              
  * @param  mode   0:5ms采样1次
                   1:1ms采样1次
  * @retval None              
  * @Note   函数调用会增加CPU开销，已经内联              
******************************************************************************
void Set_ADMode(unsigned char mode)
{
    switch(mode)
    {
        case 0:
            TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟
            flag.SampleMode=0;
            TA1CCR0 = 164;
            //TA1CCR0 = 5000;
            break; 
        case 1:                               //1ms采样1次
            TA1CTL  = TASSEL_2 + MC_1 + ID_3;        //ACLK时钟
            flag.SampleMode=1;
            TA1CCR0 = 1000;
            break;
        case 5:                    //5ms采样1次
            TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟
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


/***********************************串口***************************************/



#ifdef USART

/*******************************************************************************
  * @brief  串口初始化函数
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void usart_init( void )
{
//    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
//    DCOCTL = CALDCO_1MHZ;

/* 1MHz晶振9600波特率
    P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 |= UCSSEL_2;
    
    //UCA0BR0 = 106;                            // 1MHz 9600
    //UCA0BR1 = 0;                              // 1MHz 9600
    //UCA0BR0 = 0x03;                           // 8MHz 9600
    //UCA0BR1 = 0x00;                           // 8MHz 9600
    UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5    BRCLK/(UBR+(M7+...0)/8)
    UCA0CTL1 &= ~UCSWRST;                     // Initialize USCI state machine
*/

/* 32KHz晶振9600波特率*/
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
  * @brief  串口发送一个字节
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
  * @brief  串口接受一个字节
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
  * @brief  UART接受中断
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
  * @brief  putchar,函数重定向，自动覆盖标准库函数
  * @param  c为待发送的字符
  * @retval None
  * @Note   向串口终端发送一个字符
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
  * @brief  串口打印数据
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void print( char* str )
{
    while ( *str != '\0' ) // \0 表示字符串结束标志，通过检测是否字符串末尾
    {
        UartPutchar( *str );
        str++;
    }
}

/*******************************************************************************
  * @brief  串口打印回车换行
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
  * @brief  串口打印数字
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
  * @brief  优化串口打印数字
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
  * @brief  从FLASH设置参数              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void FLASH_LOAD()
{
    INT8U i;
    INT8U *Flash_ptr=(INT8U *)STORAGE_START_ADDR;   //数据读取指针
    for(i=0;i<sizeof(struct _storage_data);i++)
    {
        Storage_union.Storage[i]=*(Flash_ptr+i);
    }
    if(Storage_union.StorageData.Soft_Version==DEF_SOFT_VERSION) //验证版本号（flash数据是否有用是以版本号为依据的）
    {
        Report_time=Storage_union.StorageData.FLASH_REPORT_TIME;//上报时间（遥测发送间隔）
        ERROR_RSTTIME=Storage_union.StorageData.FLASH_ERROR_RSTTIME;          //故障复归时间
        TF_MAX   = Storage_union.StorageData.FLASH_TF_MAX;                 //短路突变时间
        DELT_I   = Storage_union.StorageData.FLASH_DELT_I;                  //电流突变越线阈值（短路突变电流）
        CHZ_TIME = Storage_union.StorageData.FLASH_CHZ_TIME;
        MAX_I    = Storage_union.StorageData.FLASH_MAX_I;                  //过负荷阈值
        T1_MAX   = Storage_union.StorageData.FLASH_T1_MAX;                 //除以30（每30ms一次）
        T2_MAX   = Storage_union.StorageData.FLASH_T2_MAX;
        T3_MAX   = Storage_union.StorageData.FLASH_T3_MAX;
        T4_MAX   = Storage_union.StorageData.FLASH_T4_MAX;
        VBAT_MIN = Storage_union.StorageData.FLASH_VBAT_MIN;             //电池电压低阈值
        for(i=0;i<14;i++)
        {
            SampleTable[i]=Storage_union.StorageData.FLASH_SAMPLE_TABLE[i];
        }
        
    }

}

/*******************************************************************************
  * @brief  数据保存              
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
    
    Flash_Clr((INT16S *)0x1000);           //擦除D段
    FCTL3=FWKEY;
    FCTL1=FWKEY+WRT;                       //Set WRT bit for write operation
    
   
    Storage_union.StorageData.Soft_Version        =    DEF_SOFT_VERSION;
    Storage_union.StorageData.FLASH_REPORT_TIME   =    Report_time ;         //上报时间（遥测发送间隔）
    Storage_union.StorageData.FLASH_ERROR_RSTTIME =    ERROR_RSTTIME ;     //故障复归时间
    Storage_union.StorageData.FLASH_TF_MAX        =    TF_MAX    ;          //短路突变时间
    Storage_union.StorageData.FLASH_DELT_I        =    DELT_I ;              //电流突变越线阈值（短路突变电流）
    Storage_union.StorageData.FLASH_CHZ_TIME      =    CHZ_TIME  ;
    Storage_union.StorageData.FLASH_MAX_I         =    MAX_I ;              //过负荷阈值
    Storage_union.StorageData.FLASH_T1_MAX        =    T1_MAX ;             //除以30（每30ms一次）
    Storage_union.StorageData.FLASH_T2_MAX        =    T2_MAX;        
    Storage_union.StorageData.FLASH_T3_MAX        =    T3_MAX ;       
    Storage_union.StorageData.FLASH_T4_MAX        =    T4_MAX   ;     
    Storage_union.StorageData.FLASH_VBAT_MIN      =    VBAT_MIN ;         //电池电压低阈值
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
/* 数据写入                                                                       
    FLASH_REPORT_TIME = Report_time;     //上报时间（遥测发送间隔）             
    FLASH_ERROR_RSTTIME = ERROR_RSTTIME; //故障复归时间                           
    FLASH_TF_MAX = TF_MAX;          //短路突变时间
    FLASH_DELT_I = DELT_I;             //电流突变越线阈值（短路突变电流）

    FLASH_MAX_I = MAX_I;               //过负荷阈值
    FLASH_T1_MAX= T1_MAX;               //除以30（每30ms一次）
    FLASH_T2_MAX= T2_MAX;
    FLASH_T3_MAX= T3_MAX;
    FLASH_T4_MAX= T4_MAX;
    FLASH_VBAT_MIN= VBAT_MIN; //电池电压低阈值
 */  
    FCTL1 = FWKEY;                           // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                    // Set LOCK bit
    
#ifdef USE_WATCHDOG
    WDTCTL = WDT_ARST_1000;
#endif
    _EINT();
    
}


/*******************************************************************************
  * @brief  FLASH  擦除函数              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void Flash_Clr(INT16S *Data_ptr)
{
    FCTL2 = FWKEY+FSSEL_1+FN4+FN2+FN1+FN0;       // MCLK/24 for Flash Timing Generator
    FCTL3=FWKEY;
    FCTL1 =FWKEY+ERASE;                   //启动段擦除
    *Data_ptr=0;                          //dummy  write
    
}

#endif


/*******************************************************************************
  * @brief  判据状态机
  * @param  None
  * @retval None
  * @Note   10ms完成一次采样进入一次
*******************************************************************************/
void statuscheck( INT16U value )
{
    //INT16U i;
    INT16U temp;
/* 采样间隔转换  */ 
    if(SampleDelay==0 & (P2IN&0x20)==0)//休眠(改为掉电)
    {
        //if(flag.SampleMode!=0)       //加上这个判断会有问题，但找不到原因
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
    // if(value<10)            //10ms采样1次
    // {
           // Set_ADMode(10);             
    
    // }
    else if(value<55)
    {
        //if(flag.SampleMode!=5)
        { 
            TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟
            flag.SampleMode=5;
            TA1CCR0 = 164;
        }  
    }
    else
    {
        //if(flag.SampleMode!=1)
        {
            TA1CTL  = TASSEL_2 + MC_1 + ID_3;        //SMCLK时钟
            flag.SampleMode=1;
            TA1CCR0 = 1000;
        }  
    }
	
	pre_i = last_i;
    last_i = cur_i;
    cur_i = value;

#if 0    //动态STEADY_I
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
    
//过负荷判据
    if ( cur_i > MAX_I && flag.Error!=3)       //过负荷报错
    {
        Tm++;
        if(Tm>930)       //定时10s（1000/1.075f）
        {
            Tm=0;
            fanhong();             //动作
			flag.Error = 3;
#ifdef NRF905
            nRF905_Report(0x20,(INT16U)If,0);
#endif
            status = 0;            //进入故障
            led_flash(100,4);
        }

    }
    else
    {
         Tm=0;
    }

 
//判断短路
    switch ( status )
    {
    case 0:                         //初始状态(正常启动或错误复位)
        if ( cur_i >= 1 )       //采样值大于最小正常工作电流
        {
            In_t++;    //时间计数
        }
        else
        {
            In_t = 0;    //电流为零则复位(防止非故障相重合涌动误动)
        }
        if ( In_t > 930 )          //正常运行10秒（1000/1.075f）
        {
            In_t = 0;               //清零
            if(flag.Error == 0)     //正常状态上电须保证翻白
            {
                fanbai();
#ifdef NRF905
                nRF905_Report(0x31,0,0);
#endif
                
            }
            else if ( flag.Error == 1)     //是否有永久性短路故障
            {
                RstTime_Count=0;
                fanbai();           //复归（翻牌）
				flag.Error = 0;        //错误清零
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
            f_steady = 0;           //稳定标志位清零
            normal_i = cur_i;       //获取正常运行电流
            if(normal_i<(2*DELT_I))JUGE_DELT_I=DELT_I;
                    else JUGE_DELT_I=(normal_i/2);
           
            led_flash(100, 2 );     //闪灯
#ifdef USART	
	//println("go case 1");
#endif	
            status = 1;             //进入正常运行状态
        }
        break;
    case 1:                         //正常运行
//失电归零
#if 0
        if(cur_i < DEF_In)               //归零
        {
            T_zero++;
            if(T_zero>300)          //3S掉电
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
//检测电压上升  
/**/  
        //temp=(INT16U)(last_i*0.03);
        temp=last_i>>5;
        if(temp<3)temp=3;
        if ( abs(cur_i,last_i) < temp) //2次电流差小于10视为稳定
        {
            delt_t=0;
            f_steady++;             //电流稳定标志位
            if ( f_steady > 2 )     //3次都稳定视为平直线
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
            if ( RiseEdge(cur_i,normal_i) > (INT16S)JUGE_DELT_I ) //电流变化超过电流突变越线阈值
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
        if ( absf( cur_i - last_i ) < STEADY_I ) //2次电流差小于10视为稳定
        {
            f_steady++;             //电流稳定标志位
            if ( f_steady > 2 )     //3次都稳定视为平直线
            {
                f_steady = 0;       //稳定状态清零
                if ( absf( cur_i - normal_i ) > DELT_I ) //稳定电流变化值超过电流突变越线阈值
                {
                    Tf = 3;                  //数字代表10ms
                    status = 2;              //进入高电流状态
                }
                else
                {
                    normal_i=cur_i;
                }

            }
        }

        else                             //电流变化较大
        {
            f_steady = 0;
            if ( ( cur_i - normal_i ) > DELT_I ) //电流变化超过电流突变越线阈值（适配手动加电）
            {
                Tf = 0;
                status = 2;
            }
            else
            {
                delt_t++;                //电流变化时间+1
            }
        }
*/
        break;
    case 2:                            //突变（高电流状态）
        //uart_normal_i=1;
        if ( abs(cur_i,last_i ) < STEADY_I ) //2次电流差小于10视为稳定
        {
            f_steady++;                //电流稳定标志位，
            if ( f_steady > 2 )        //3次都稳定视为平直线
            {
                f_steady = 0;          //稳定状态清零
                if ( RiseEdge(cur_i,normal_i ) > (INT16S)JUGE_DELT_I ) //稳定电流变化值超过100
                //if ( abs(cur_i,normal_i ) > DELT_I )
                {
                    //Tf += 3;
                    if(cur_i>If)If=cur_i;
                }
                else if ( cur_i >= DEF_In )
                {
                    status = 1;        //负荷波动防误动
                }
                else if ( cur_i < DEF_In ) //电流为零
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
        
        if (abs(cur_i,normal_i) > (JUGE_DELT_I-10) ) //仍然在高电流（也是手动适配）
      //if (abs(cur_i,normal_i) > DELT_I ) //仍然在高电流（也是手动适配）
        {
            if(cur_i>If)If=cur_i;
            Tf++;
#if 1 //大电流掉电BUG测试
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
            if(Ts>CHZ_TIME)        //无重合闸
            {
                Ts=0;
                fanhong();           //动作
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
                fanhong();           //动作
				flag.Error = 4;
#ifdef NRF905
                nRF905_Report(0x16,(INT16U)If,0);
                If=0;
#endif
                
                status = 0;
                led_flash(100, 4 );

            }
                
            /*
            if ( absf( cur_i - last_i ) < STEADY_I ) //2次电流差小于10视为稳定
            {
                f_steady++;             //电流稳定标志位
                if ( f_steady > 2 )     //3次都稳定视为平直线
                {
                    normal_i=cur_i;
                    //Tc+=3;
                }
            }
            */
        }
        else if(cur_i<DEF_In)         //归零（永久故障）
        {
            fanhong();           //动作
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

//判断接地

switch ( flag.jiedi )       //接地判据
{
    case 0:
        if ( abs(cur_i,last_i) < STEADY_I ) //2次电流差小于10视为稳定
        {
            Groundsteady++;             //电流稳定标志位
            if ( Groundsteady > 2 )     //3次都稳定视为平直线
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
			if ( abs(cur_i,last_i) < STEADY_I ) //2次电流差小于10视为稳定
			{
				Groundsteady++;             //电流稳定标志位
				if ( Groundsteady > 2 )     //3次都稳定视为平直线
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
                fanhong();             //动作
				flag.Error = 2;
#ifdef NRF905
                nRF905_Report(0x18,Ig-maichong_i,0);
#endif
                
                //status = 0;            //进入故障
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
/* T1 T2 T3 T4 串口输出      
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
