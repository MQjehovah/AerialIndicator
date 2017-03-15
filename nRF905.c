/*******************************************************************************
  * @file                   nRF905.c
  * @Author:                MQjehovah                 mail:MQjehovah@hotmail.com
  * @version                1.0.0
  * @date                   2016.4.7
  * @brief                  FOR MSP430
  ******************************************************************************
  * @attention
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "nRF905.h"

/* Definition ----------------------------------------------------------------*/
const unsigned char ADDR_TABLE[] @0x1000 = {
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
nRF905_ADDR        ,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};
char TxBuf[TxRxBuf_Len];
char RxBuf[TxRxBuf_Len];

unsigned char RFConf[11];           //NRF905：10寄存器配置 


char TxAddress[4]={0xFE,0xDC,0xBA,0x00};//发送地址0x55,0x00,0x01,0x11
char DATA_BUF;

extern flagbit flag;

//判据数据
extern INT16U If;
//extern float Tf;
extern INT16U normal_i;
extern INT16U cur_i;
extern INT16U DELT_I;
extern INT16U TF_MAX;
extern INT16U MAX_I;

extern INT32U Report_time;
extern INT32U RstTime_Count;
extern INT32U ERROR_RSTTIME;
extern INT16U T1_MAX;   
extern INT16U T2_MAX;
extern INT16U T3_MAX;
extern INT16U T4_MAX;
extern INT16U VBAT_MIN;
extern INT16U CHZ_TIME;
extern INT16U PUL_MIN_I;

extern INT16U VBAT;
extern INT16U CPU_TEMP;

extern unsigned char status; 

extern INT32U RX_DELAY;
extern INT32U PRE_RX_DELAY;


extern INT16U SampleDelay;
extern INT32U oldSampleValue;

extern INT16U SampleTable[14];
extern INT16U CurTable[14];

extern float  cur_k[13];
/* Functions -----------------------------------------------------------------*/
#ifdef USE_NOP
/*******************************************************************************
  * @brief  延时            
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
void Delaynop()
{
	delay_us(10);
}
#endif
/*******************************************************************************
  * @brief  NRF905 SPI读函数            
  * @param  None              
  * @retval None              
  * @Note   IO模拟SPI时序              
*******************************************************************************/
unsigned char SpiRead(void)
{
	unsigned char i;
	for (i=0;i<8;i++)
	{
		DATA_BUF=DATA_BUF<<1;
#ifdef USE_NOP
		Delaynop();
#endif
		SCK_1;
#ifdef USE_NOP
		Delaynop();
#endif
		if ((P3IN&0x04))	//读取最高位，保存至最末尾，通过左移位完成整个字节
		{
			DATA_BUF|=0x01;
		}
		else
		{
			DATA_BUF&=~(0x01);
		}
#ifdef USE_NOP
		Delaynop();
#endif
		SCK_0;
#ifdef USE_NOP
		Delaynop();
#endif
    }
	return DATA_BUF;
}
/*******************************************************************************
  * @brief  NRF905 SPI写函数
  * @param  None              
  * @retval None              
  * @Note   IO模拟SPI时序              
*******************************************************************************/
void SpiWrite(unsigned char send)
{
	unsigned char i;
	DATA_BUF=send;
	for (i=0;i<8;i++)
	{
		if (((DATA_BUF&0x80) != 0))	//总是发送最高位 
        {
			MOSI_1;
		}
		else
		{
			MOSI_0;
		}
		SCK_1;
#ifdef USE_NOP
		Delaynop();
#endif
		DATA_BUF=DATA_BUF<<1;
		SCK_0;
#ifdef USE_NOP
		Delaynop();
#endif

	}
}
/*******************************************************************************
  * @brief  nRF905引脚配置          
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void nRF905_IO_set(void)
{
    //P3DIR |= 0xeb; P3DIR &= 0xfb;
    P3DIR |= (BIT0+BIT1+BIT3+BIT4+BIT5+BIT6+BIT7);
    P3DIR &= ~BIT2;

	P2DIR &= ~BIT3;  //DR
    P3REN |= BIT2;   //输入端口MISO
    P2REN |= BIT3;   
    
    CSN_1;        // Spi  disable
    SCK_0;        // Spi clock line init low
    PWR_0;        // nRF905 power on
    TRX_CE_0;     // Set nRF905 in standby mode
    TXEN_0;       // set radio in Rx mode

    flag.nrf905_mode=1;//905休眠状态
}
/*******************************************************************************
  * @brief  初始化NRF905     
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void Config905(void)
{
	uchar i;
	CSN_0;						// Spi enable for write a spi command
#ifdef USE_NOP
	Delaynop();
#endif
	for (i=0;i<11;i++)			// Write configration words  写放配置字
	{
	   SpiWrite(RFConf[i]);
	}
#ifdef USE_NOP
	Delaynop();
#endif
	CSN_1;						//关闭SPI

}
/*******************************************************************************
  * @brief  NRF905装载地址+数据打包+数据发送 
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void TxPacket(char *data_Bufer)
{
	uchar i;
//	TRX_CE_0;
#ifdef USE_NOP
	Delaynop();
#endif
	CSN_0;
#ifdef USE_NOP
	Delaynop();		
#endif		
	SpiWrite(WTP);              // 待发数据装载命令
	for (i=0;i<TxRxBuf_Len;i++)
	{
	   SpiWrite(data_Bufer[i]);
	}
	CSN_1;                      // 关闭SPI
#ifdef USE_NOP
	Delaynop();
#endif
	CSN_0;						// 打开SPI
#ifdef USE_NOP
	Delaynop();
#endif
	SpiWrite(WTA);				// 写入地址要和接收方地址一样
	for (i=0;i<4;i++)			// 4字节地址
	{
	  SpiWrite(TxAddress[i]);
	}
#ifdef USE_NOP
	Delaynop();
#endif
	CSN_1;		//关闭SPI
#ifdef USE_NOP
	Delaynop();
#endif
	TRX_CE_1;					// Set TRX_CE high,start Tx data transmission
//	Delaynop();					// while (DR!=1); 延时时间不能太短
//	while(CheckDR()!=1);
    delay_ms(2);
	TRX_CE_0;					// Set TRX_CE low
}
/*******************************************************************************
  * @brief  发送模式初始化
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void SetTxMode(void)
{

    if(flag.nrf905_mode==0)//从掉电重启
    {
        CSN_1;        // Spi  disable
        SCK_0;        // Spi clock line init low
        PWR_1;        // nRF905 power on
        TRX_CE_0;     // Set nRF905 in standby mode
        TXEN_0;       // set radio in Rx mode
        delay_ms(10);
        Config905();
        
    }

	TRX_CE_0;
	TXEN_1;
	delay_ms(1); 					// Delay for mode change(>=650us)
    
    flag.nrf905_mode=2;             //905发送状态
    
}
/*******************************************************************************
  * @brief  DR检测,当 收到数据后DR置1，当把数据读出来后DR清0
  * @param  None              
  * @retval None              
  * @Note   None               
*******************************************************************************/
unsigned char CheckDR(void)		//检查是否有新数据传入 Data Ready
{
	if ((P2IN&0x08))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/*******************************************************************************
  * @brief  设置接收模式
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void SetRxMode(void)
{
	TXEN_0;
	TRX_CE_1;
	delay_ms(1); 		// delay for mode change(>=650us)
    
    flag.nrf905_mode=3;     //905接收状态
    RX_DELAY=PRE_RX_DELAY;
}

/*******************************************************************************
  * @brief  数据接收
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void RxPacket(void)						
{
    char i;
    delay_ms(1);
//	delay_ms(10);
    TRX_CE_0;
    CSN_0;							// SPI使能（Spi enable for write a spi command）
	delay_ms(1);
    SpiWrite(RRP);					// 读SPI数据命令（Read payload command）
	for (i = 0 ;i <14 ;i++)
    {  
      RxBuf[i]=SpiRead();			// Read data and save to buffer    
    }
    CSN_1;
    delay_ms(1);
    TRX_CE_1;							
}
/*******************************************************************************
  * @brief  NRF905数据接收流程
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void RX(void)
{
    SetRxMode();			// Set nRF905 in Rx mode
//	while (CheckDR()==0);
    delay_ms(5);
    RxPacket();		// Recive data by nRF905
    delay_ms(5);
}
/*******************************************************************************
  * @brief  NRF905初始化函数
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void nRF905Init(void)
{
        TxBuf[0]=ADDR_TABLE[144];
        TxBuf[1]=ADDR_TABLE[145];
        TxBuf[2]=ADDR_TABLE[146];
        TxBuf[3]=ADDR_TABLE[147];

	RFConf[0]= 0x00;
	RFConf[1]= 0x4c;
	RFConf[2]= 0x0C;
	RFConf[3]= 0x44;
	RFConf[4]= 0x0e;
	RFConf[5]= 0x0e;
	RFConf[6]= ADDR_TABLE[144];
	RFConf[7]= ADDR_TABLE[145];
	RFConf[8]= ADDR_TABLE[146];
	RFConf[9]= ADDR_TABLE[147];
	RFConf[10]= 0x58;

	nRF905_IO_set();
	delay_ms(200);
	Config905();
}



/*******************************905数据处理************************************/



/*******************************************************************************
  * @brief  905发送数据
  * @param  None              
  * @retval None              
  * @Note   重复3次            
******************************************************************************
void nRF905_send()
{
	unsigned char i;
	for(i=0;i<4;i++)
	{
		SetTxMode();
		TxPacket(TxBuf);
		SetRxMode();
        delay_ms(160);
	}	 
}
*/
/*******************************************************************************
  * @brief  设置发送数据
  * @param  Report_num：上报序号
            Report_data：上报数据，没有为0
            mode:   0--上报32次等待接收（交给主程序发送）
                    1--上报3次（等待发送）              
  * @retval None              
  * @Note   设置上报数据
*******************************************************************************/
void nRF905_Report(unsigned char Report_num ,INT16U Report_data,unsigned char mode)
{
    unsigned char i;
    if(Report_num!=0x30)
    Tx_clear();
	
    TxBuf[4]=Report_num;
    TxBuf[5]=Report_num;
    TxBuf[6]=((INT16U)(Report_data))/256;
    TxBuf[7]=((INT16U)(Report_data))%256;
	
    TxBuf[13]=flag.Error;
    SampleDelay=DEF_SAMPALE_DELAY;//采样休眠延时重置(唤醒CPU)
    flag.SampleMode=5;
	
	
    if(mode==0)           //连续发送32次，收到返回则停止
    {
        flag.SendCount = SEND_TIMES;
    }
    else if(mode ==1)     //只发送3次
    {
        for(i=0;i<4;i++)
        {
            SetTxMode();
            TxPacket(TxBuf);
            SetRxMode();
            delay_ms(160);
        }	         
    }


}

/*******************************************************************************
  * @brief  清空发送数据
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/
void Tx_clear()
{
    int i;
	
    for ( i= 4; i<14; i++) 
    {
          TxBuf[i]=0;
    }
   // TxBuf[4]=0x01;
}

/*******************************************************************************
  * @brief  None              
  * @param  None              
  * @retval None              
  * @Note   None              
*******************************************************************************/
INT16U RX_Recive(unsigned char mode)
{
    INT16U temp;
    if(mode==0)
    {
        temp=(INT16U)RxBuf[10];
        temp<<=8;
        temp+=(INT16U)RxBuf[11];
        return temp;
    }
    else
    {
        temp=(INT16U)RxBuf[12];
        temp<<=8;
        temp+=(INT16U)RxBuf[13];
        return temp;
    }

}

/*******************************************************************************
  * @brief  接收数据处理
  * @param  None              
  * @retval None              
  * @Note   None             
*******************************************************************************/ 
void RX_Proc()
{
    INT16U temp;
    unsigned char Ih=0;
    unsigned char Il=0;
    
	if(RxBuf[4]==0xA5 & RxBuf[5]==0x5A)
	{
        if(RxBuf[7]==0x00)                  //设置命令
        {
        //    Tx_clear();
            switch(RxBuf[8])
            {
                case 0x01:                  //遥测发送间隔
                    Report_time=RX_Recive(0)*(INT32U)1000;
                    led_flash(100,1);
                    break;
                    
                case 0x02:                  //故障复归时间
                    ERROR_RSTTIME=RX_Recive(0)*60000;
                    led_flash(100,1);
                    break;

                case 0x03:                  //短路突变时间
                    TF_MAX=RX_Recive(0)/10;
                    led_flash(100,1);
                    break;	

                case 0x04:                  //短路突变电流
                    DELT_I=RX_Recive(0);
                    led_flash(100,1);
                    
                    break;

                case 0x05:                  //重合闸时间
                    CHZ_TIME=RX_Recive(0)/10;
                    led_flash(100,1);
                    break;

                case 0x06:                  //过负荷阈值
                    MAX_I=RX_Recive(0);
                    led_flash(100,1);
                    break;
  
                case 0x07:
                    T1_MAX=RX_Recive(0)/10;//除以10（每10ms一次）???
                    led_flash(100,1);
                    break;

                case 0x08:
                    T2_MAX=RX_Recive(0)/10;
                    led_flash(100,1);
                    break;

                case 0x09:
                    T3_MAX=RX_Recive(0)/10;
                    led_flash(100,1);
                    break;
                    
                case 0x0A:
                    T4_MAX=RX_Recive(0)/10;
                    led_flash(100,1);
                    break;	

                case 0x0B:
                    VBAT_MIN=RX_Recive(0);
                    led_flash(100,1);
                    break;	 
                    
                case 0x0F:
                    temp=RX_Recive(0);
                    switch(temp)
                    {
						case 6:
							if(oldSampleValue>1)
								SampleTable[0]=oldSampleValue;
							else
								SampleTable[0]=2;
							break;
                        case 10:
                            SampleTable[1]=oldSampleValue;
                            break;
                        case 50:
                            SampleTable[2]=oldSampleValue;
                            break;
                        case 80:
                            SampleTable[3]=oldSampleValue;
                            break;
                        case 100:
                            SampleTable[4]=oldSampleValue;
                            break;
                        case 150:
                            SampleTable[5]=oldSampleValue;
                            break;
                        case 200:
                            SampleTable[6]=oldSampleValue;
                            break;
                        case 250:
                            SampleTable[7]=oldSampleValue;
                            break;
                        case 300:
                            SampleTable[8]=oldSampleValue;
                            break;
                        case 350:
                            SampleTable[9]=oldSampleValue;
                            break;
                        case 400:
                            SampleTable[10]=oldSampleValue;
                            break;
                        case 500:
                            SampleTable[11]=oldSampleValue;
                            break;
                        case 600:
                            SampleTable[12]=oldSampleValue;
                            break;
                        case 630:
                            SampleTable[13]=oldSampleValue;
                            break;
                    }

                    k_cacl();
                    if(temp==6 || temp==10 || temp==50)
                    {
                        Table1_50_cacl();
                    }    
					
                    nRF905_Report(0x40,0,1);	
					fanhong();					
                    break;	

            }
        
		//led_flash(100,1);
		}
        else if(RxBuf[7]==0x01)              //读取命令
        {
        //    Tx_clear();
            switch(RxBuf[8])
            {
#if 0
                case 0x00:                   //软件版本
                    nRF905_Report(0x50,DEF_SOFT_VERSION,1);
                    break;
                case 0x01:                   //遥测发送间隔
                    nRF905_Report(0x51,(INT16U)(Report_time/1000),1);
                    break;
                    
                case 0x02:                  //故障复归时间
                    nRF905_Report(0x52,(INT16U)(ERROR_RSTTIME/60000),1);
                    break;

                case 0x03:                   //短路突变时间
                    nRF905_Report(0x53,TF_MAX * 10,1);
                    break;	

                case 0x04:                   //短路突变电流
                    nRF905_Report(0x54,DELT_I,1);
                    break;

                case 0x05:                   //重合闸时间
                    nRF905_Report(0x55,CHZ_TIME*10,1);//???
                    break;

                case 0x06:                   //过负荷阈值
                    nRF905_Report(0x56,MAX_I,1);
                    break;

                case 0x07:                  
                    nRF905_Report(0x57,T1_MAX*10,1);//???
                    break;

                case 0x08:
                    nRF905_Report(0x58,T2_MAX*10,1);//???
                    break;

                case 0x09:
                    nRF905_Report(0x59,T3_MAX*10,1);//???
                    break;	
               
                case 0x0A:
                    nRF905_Report(0x59,T4_MAX*10,1);//???
                    break;	

                case 0x0B:
                    nRF905_Report(0x59,VBAT_MIN,1);//???
                    break;	
#endif     
                case 0x10:
                    nRF905_Report(0x50,DEF_SOFT_VERSION,1);
                    
                    nRF905_Report(0x51,(INT16U)(Report_time/1000),1);
                    
                    nRF905_Report(0x52,(INT16U)(ERROR_RSTTIME/60000),1);
                    
                    nRF905_Report(0x53,TF_MAX * 10,1);
                    
                    nRF905_Report(0x54,DELT_I,1);
                    
                    nRF905_Report(0x55,CHZ_TIME*10,1);
                    
                    nRF905_Report(0x56,MAX_I,1);
                    
                    nRF905_Report(0x57,T1_MAX*10,1);
                    
                    nRF905_Report(0x58,T2_MAX*10,1);
                    
                    nRF905_Report(0x59,T3_MAX*10,1);
                    
                    nRF905_Report(0x5A,T4_MAX*10,1);
                    
                    nRF905_Report(0x5B,VBAT_MIN,1);
                    
                    break;
            }
        }
        else if(RxBuf[7]==0x02)              //遥控命令
        {
        //    Tx_clear();
            switch(RxBuf[8])
            {
                case 0x01:
                    fanbai();
                    break;
                case 0x02:
                    fanhong();
                    break;
                case 0x03:
                    flag.jiedi = 0;
                    fanhong();             //动作
                    nRF905_Report(0x18,0,0);
                    flag.Error = 2;
                    status = 0;            //进入故障
                    led_flash(100, 4 );
                    break;
                case 0x04:
                    TxBuf[8]=((INT16U)(VBAT))/256;
                    TxBuf[9]=((INT16U)(VBAT))%256; 

                    //TxBuf[10]=((INT16U)(CPU_TEMP))/256;
                    //TxBuf[11]=((INT16U)(CPU_TEMP))%256;        
                    TxBuf[10]=flag.TempSymbol;
					TxBuf[11]=((INT8U)(CPU_TEMP));	
		
                    TxBuf[13]=flag.Error;
                    nRF905_Report(0x30,(INT16U)cur_i,0);//遥测

                    break;
                case 0x05:
                    RstTime_Count=0;    //复归时间计数清零
                    fanbai();           //复归（翻牌）
                    nRF905_Report(0x17,0,0);
                    flag.Error = 0;     //错误清零
                    break;
                case 0x06:
                    fanhong();           //动作
                    nRF905_Report(0x19,(INT16U)If,0);
                    flag.Error = 1;
                    status = 0;
                    led_flash(100, 4 );
                    break;
                case 0x07:
                    FLASH_SAVE();
                    nRF905_Report(0x40,0,1);
                    break;
            }
        }
	}
    
	if(RxBuf[0]==0xFE & RxBuf[1]==0xDC & RxBuf[2]==0xBA & RxBuf[3]==0x00)
	{
		flag.SendCount=0;
        if(RxBuf[13]==0x01)PRE_RX_DELAY=30000;
        if(RxBuf[13]==0x02)PRE_RX_DELAY=1200000;
        SetRxMode();
	}

}


/*********************************END OF FILE**********************************/
