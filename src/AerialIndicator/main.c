/*******************************************************************************
  * @file                   main.c
  * @Author:                MQjehovah                 mail:MQjehovah@hotmail.com
  * @version                1.0.0
  * @date                   2016.4.6
  * @brief                  FOR MSP430
  ******************************************************************************
  * @attention
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "msp430x21x2.h"
#include "nRF905.h"
#include "stdio.h"
#include "Drivers.h"
#include <math.h>

/* Definition ----------------------------------------------------------------*/
//导入全局变量
extern flagbit flag;

extern char TxBuf[TxRxBuf_Len];
extern char RxBuf[TxRxBuf_Len];

extern unsigned char status;

extern INT32U SampleValue;
extern INT16U SampleValue1;

extern INT32U oldSampleValue;
extern INT32U SampleBuffer[10];

extern INT16U VBAT_MIN;

extern INT16U VBAT;
extern INT16U CPU_TEMP;

extern INT16U SEND_DELAY;
extern INT16U SOFT_WDG;
extern INT32U RX_DELAY;   
extern INT16U PWRDN_DELAY; 
extern INT32U PRE_RX_DELAY;
//50A以下电流系数校正采用查表法减少计算量
INT8U SampleTable_1_50A[50] = {    0, 1, 1, 1, 1,           1, 6, 8, 11, 14,
                                   17, 20, 25, 28, 31,      33, 35, 41, 45, 48,
                                   52, 55, 60, 62, 66,      70, 72, 76, 80, 84,
                                   86, 90, 94, 98, 101,     105, 108, 112, 116, 120,
                                   125, 127, 130, 135, 138, 141, 145, 148, 151, 155
                                   //,160, 170, 180, 190, 200
								   };

//电流系数校正采样值
INT16U SampleTable[14] = {4, 17, 156, 215, 267, 393, 494, 572, 624, 659, 686, 726, 756, 1024};
//电流系数校正电流值
INT16U CurTable[14] = {6, 10, 50, 80, 100, 150, 200, 250, 300, 350, 400, 500, 600, 1940};
//电流系数校正斜率
float  cur_k[13] = {0};
/*******************************************************************************
  * @brief  计算斜率
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void k_cacl()
{
    unsigned char i;
    for ( i = 0; i < 13; i++ )
    {
        cur_k[i] = ( float )( CurTable[i + 1] - CurTable[i] ) / ( float )( SampleTable[i + 1] - SampleTable[i] );
    }
	
}


/*******************************************************************************
  * @brief  计算1-50A电流采样表
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
void Table1_50_cacl()
{
#if 1 //数组在内存中
    unsigned char i;
	INT16U Sample_6A=SampleTable[0];
    float temp = 1 / cur_k[0];

	i=5;
	while(Sample_6A>1)
	{
		Sample_6A--;
		if(i>1)SampleTable_1_50A[i--]=Sample_6A;
	}
	while(i>1) 
	{
		SampleTable_1_50A[i--]=1;
	}


	temp = 1 / cur_k[0];
    for ( i = 6; i < 10; i++ )
    {
        SampleTable_1_50A[i] = ( INT8U )( SampleTable[0] + ( i - CurTable[0] ) * temp );
    }
	
	temp = 1 / cur_k[1];
	for(i=10;i<50;i++)
	{
		SampleTable_1_50A[i] = ( INT8U )( SampleTable[1] + ( i - CurTable[1] ) * temp );
	}
	
	
#if 0	
	temp = 1 / cur_k[2];
	for(i=50;i<55;i++)
	{
		SampleTable_1_50A[i] = ( INT8U )( SampleTable[2] + ( i - CurTable[2] ) * temp );
	}
#endif

#endif


#if 0  //数组在flash中（ram空间不足时可以考虑将数据放入flash中）
    unsigned char i;
    INT8U* Flash_ptr = ( INT8U* )( &SampleTable_1_50A );
    float temp = 1 / cur_k[0];
    for ( i = 15; i < 50; i++ )
    {
        *( Flash_ptr + i ) = ( INT8U )( SampleTable[0] + ( i - CurTable[0] ) * temp );
    }
#endif
	

}

/* Functions -----------------------------------------------------------------*/
/*******************************************************************************
  * @brief  系统初始化
  * @param  None
  * @retval None
  * @Note   配置时钟
*******************************************************************************/
void Init_System()
{
    //-------------设置看门狗定时器---------------------------//
    WDTCTL = WDTPW + WDTHOLD;          // Stop watchdog timer
    /*时钟设置*/
    if ( CALBC1_8MHZ == 0xFF || CALDCO_8MHZ == 0xFF ) //读取信息段配置信息
    {
        while ( 1 );                   // If calibration constants erased
        // do not load, trap CPU!!
    }
#ifdef CPU_1MHZ
    BCSCTL1 = CALBC1_1MHZ;             // Set DCO to 1MHz
    DCOCTL = CALDCO_1MHZ;
#endif
#ifdef CPU_8MHZ
    BCSCTL1 = CALBC1_8MHZ;             // Set DCO to 8MHz
    DCOCTL = CALDCO_8MHZ;
#endif
    _EINT();
}

/*******************************************************************************
  * @brief  电流值系数修正 V2.0.0
  * @param  None
  * @retval None
  * @Note   将采样值转为对应电流值
            k=(ElecEnd-ElecStart)/(SampleEnd-SampleStart)
            elec=ElecStart+(SampleValue-SampleStart)*k
*******************************************************************************/
INT16U check_elec( INT32U SampleValue )
{
    INT8U i = 0;
    if ( SampleValue <= SampleTable_1_50A[6])
    {
        i=6;
        while ( SampleTable_1_50A[i] >SampleValue )
        {
          i--;
          if (0==i)
            return 0;
        };
        return i;
    }
    else
    if ( SampleValue <= SampleTable_1_50A[49] )
    {
        i=7;
        while ( SampleTable_1_50A[i] <SampleValue )
        {
          i++;
          if(i>=50)  break;
        }
        return i;
    }
    else
    {
        while ( SampleValue > SampleTable[i] )
        {
          i++;
          if(i>=14)  break;
        }  
        i++; 
        return ( INT16U )( CurTable[i - 2] + ( SampleValue - SampleTable[i - 2] ) * cur_k[i - 2] );
    }
}

/*******************************************************************************
  * @brief  主函数
  * @param  None
  * @retval None
  * @Note   None
*******************************************************************************/
int main( void )
{
    //int i = 0;
    INT16U SampleTemp;//临时暂放电池电压采样和CPU温度采样值做进一步转换
    INT16U SampleTemp1;
	float temp;
    /* 系统初始化 */
    Init_System();
    gpio_init();
#ifdef USART
    usart_init();
#endif

#ifdef NRF905
    nRF905Init();
#endif

#ifdef USE_ADC
    adc_init();
#endif

    led_init();    //LED初始化

    /* 翻牌 */
    flop_init();    //翻牌初始化
    fanbai();       //初始化翻白牌

#ifdef USE_FLASH
    FLASH_LOAD();
#endif

    k_cacl();         //计算斜率
    Table1_50_cacl(); //计算1-50A电流采样表
    flag.SampleMode = 5; //初始化5ms一次采样率
    /* 定时器 */
#ifdef USE_TIMERA0 //开系统计时定时器
    TimerA0_Init();
    Start_Timer_A0();
#endif

#ifdef USE_TIMERA1 //开采样唤醒定时器
    TimerA1_Init();
    Start_Timer_A1();
#endif

#ifdef NRF905
    SetRxMode();   //905置为接受模式
#endif

#ifdef USART        //串口调试信息
    println( "START" );
#endif

#ifdef USE_WATCHDOG    //看门狗
    WDTCTL = WDT_ARST_1000;
#endif


    exit_init();    //比较器中断
//compare_init();

    /* 闪灯 */
    led_flash( 500, 2 ); //程序运行指示灯

//开始采样
/*
    flag.RequireTemp = 0;
    flag.RequireVBAT = 0;
    flag.NRF_Recived = 0;
    flag.SendCount = 0;
*/	
    flag.RequireTemp = 1;
    flag.RequireVBAT = 1;
    flag.Sample_EN = 1;
#ifdef USE_SOFT_WDG
	SOFT_WDG=DEF_SOFT_WDG_TIME;
	flag.Soft_WDG_EN=1;
#endif
    /* 主循环 */
    while ( 1 )
    {
        if ( flag.SampleMode == 1 ) //不同采样率不同休眠方式（1ms采样率使用SMCLK时钟如果进入LPM3休眠会停止SMCLK时钟）
        {
            LPM0;
        }
        else
        {
            LPM3;
        }
#ifdef USE_SOFT_WDG
		SOFT_WDG=DEF_SOFT_WDG_TIME;//软件看门狗喂狗程序
#endif
		
        flag.Sample_EN = 0; //主循环中关闭采样（如果开启可能可以不必做Tf和接地时间的补偿）
        Stop_Timer_A1();
        /* 905接收 */
#ifdef NRF905
        if ( CheckDR() ) //检查DR引脚判断905接收
        {
            RX();      //接收数据
            flag.NRF_Recived = 1;
        }
#endif


        /* 采样值处理 */
        if ( flag.SampleReady == 1 )
        {
            flag.SampleReady = 0;
#ifdef USART
            //printNumber1((long)check_elec(oldSampleValue),10);
            //printNumber1((long)oldSampleValue,10);
#endif
            statuscheck( ( INT16U )check_elec( oldSampleValue ) ); //调用判据函数

        }
        /* 请求温度采样 */
        if ( flag.RequireTemp == 1 )
        {
            flag.RequireTemp = 0;
            //CPU_TEMP=adc_get_value(10);
            //CPU_TEMP=SingleSample(10);
            SampleTemp1 = SingleSample( 10 );     //AD采样10通道采CPU内部温度
			temp=( float )SampleTemp1 * 0.4126f;
			if(temp>=277.75f)
			{
				flag.TempSymbol=0;
				CPU_TEMP = ( INT16U )(temp - 277.75f );
			}
			else
			{
				flag.TempSymbol=1;
				CPU_TEMP = ( INT16U )(277.75f - temp);
			}
             //温度系数转换
#ifdef USART
            //printNumber1((long)CPU_TEMP,10);
#endif
        }
        /* 请求电池电压采样 */
        if ( flag.RequireVBAT == 1 )
        {
            flag.RequireVBAT = 0;
            // VBAT=adc_get_value(11);
            SampleTemp = SingleSample( 11 ); //AD采样11通道采VCC电压
            if ( SampleTemp < 546 )
            {
                VBAT = 330;    //电池电压低于3.4V后，进二极管只剩2.7V是CPU内部稳压器工作的最低电压
            }
            else
            {
                VBAT = ( INT16U )( ( ( SampleTemp - 546 ) * 5 + 3400 ) / 10 );    //电池电压系数转换
            }

#ifdef USART
            //printNumber1((long)VBAT,10);
#endif
            /* 电池电压低报警 */
            if ( VBAT < VBAT_MIN )           //电池电压低报警
            {
                if ( flag.SendCount == 0 && flag.VBAT_Low == 0 )
                {
                    flag.VBAT_Low = 1;
                    nRF905_Report( 0x15, 0, 0 );
                }
            }
#if 0       //电池电压恢复
            else
            {
                if ( flag.VBAT_Low == 1 )
                {
                    flag.VBAT_Low = 0;
                }
            }
#endif
#ifdef USART
            // printNumber(adc_get_value(11));
#endif
        }

#ifdef USART             //主程序串口发送
        if ( flag.usart_send == 1 )
        {
            flag.usart_send = 0;
#ifdef USART
            //printNumber1(uart_max_i,10);
#endif

        }
#endif


#ifdef NRF905
        /* 接收处理 */
        if ( flag.NRF_Recived == 1 )
        {
            flag.NRF_Recived = 0;
            RX_Proc();//905接收数据处理

        }
#endif

#ifdef NRF905
        /* 905发送 */
        if ( flag.SendCount > 0 & SEND_DELAY == 0 )
        {
            SetTxMode();
            TxPacket( TxBuf );
            SetRxMode();
            SEND_DELAY = DEF_SEND_DELAY; //控制2次发送间隔必须大于SEND_DELAY
            //delay_ms(150);
            flag.SendCount--;

        }
#endif

/* 休眠判断 */
		if((flag.PowerDown==1) & (flag.SendCount==0) )
		{
			status=0;
			flag.PowerDown=0;
			TA1CTL  = TASSEL_1 + MC_1;        //ACLK时钟
            flag.SampleMode=0;
//关断905
			flag.SendCount = 0;
            RX_DELAY=0;
            TRX_CE_0;
            PWRDN_DELAY=0;
            PWR_0;
            PRE_RX_DELAY=DEF_RX_DELAY;//重设休眠延时预设值
            flag.nrf905_mode=0;//掉电模式

            TA1CCR0 = 164;  
		}
		
        //delay_ms(1000);
        flag.Sample_EN = 1;
        Start_Timer_A1();
    }
    return 0;
}





/*********************************END OF FILE**********************************/
