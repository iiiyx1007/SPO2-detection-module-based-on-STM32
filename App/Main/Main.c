/*********************************************************************************************************
* 模块名称：Main.c
* 摘    要：主文件，包含软硬件初始化函数和main函数
* 当前版本：1.0.0
* 作    者：SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* 完成日期：2020年01月01日
* 内    容：
* 注    意：注意勾选Options for Target 'Target1'->Code Generation->Use MicroLIB，否则printf无法使用                                                                  
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Main.h"
#include "stm32f10x_conf.h"
#include "DataType.h"
#include "NVIC.h"
#include "SysTick.h"
#include "RCC.h"
#include "Timer.h"
#include "UART1.h"
#include "LED.h"
#include "Wave.h"
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "DAC.h"
#include "SendDataToHost.h"
#include "ADC.h"
#include "ECG.h"
#include "spo2check.h"
#include "Filiter.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define RED_CS_PIN       GPIO_Pin_7
#define RED_CS_PORT      GPIOB
#define IR_CS_PIN        GPIO_Pin_6
#define IR_CS_PORT       GPIOB
/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
//int heartrate = 0;
extern u16 s_arrADC1Data;
int pulseRate = 0;
int Spo2 = 0;
int led_state = 0;
u16 receive_cnt = 0;
u16 DACdate = 100;
u16 IRwave,REDwave,iravg,redavg,IR_wavedata, RED_wavedata;
extern float arr_RedFilter[1500]; //红光滤波数组
extern float arr_IrFilter[1500];  //红外光滤波数组
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
extern MovingAverageFilter IR_movingFilter;
extern MovingAverageFilter RED_movingFilter;

extern Filter              IR_HighPass;
extern Filter              RED_HighPass;
extern Filter              IR_LowPass;
extern Filter              RED_LowPass;
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  InitSoftware(void);   //初始化软件相关的模块
static  void  InitHardware(void);   //初始化硬件相关的模块
static  void  Proc2msTask(void);    //2ms处理任务
static  void  Proc1SecTask(void);   //1s处理任务
static  void  Proc500usTask(void);  //500us处理任务
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitSoftware
* 函数功能：所有的软件相关的模块初始化函数都放在此函数中
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  InitSoftware(void)
{
  InitPackUnpack();       //初始化PackUnpack模块
//  InitProcHostCmd();      //初始化ProcHostCmd模块
  InitSendDataToHost();   //初始化SendDataToHost模块
}

/*********************************************************************************************************
* 函数名称：InitHardware
* 函数功能：所有的硬件相关的模块初始化函数都放在此函数中
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  InitHardware(void)
{  
  SystemInit();       //系统初始化
  InitRCC();          //初始化RCC模块
  InitNVIC();         //初始化NVIC模块
  InitUART1(115200);  //初始化UART模块
  InitTimer();        //初始化Timer模块
  InitLED();          //初始化LED模块
  InitSysTick();      //初始化SysTick模块
  InitDAC();          //初始化DAC模块
  InitADC();          //初始化ADC模块
	Init_ECG();         //初始化ECG模块
	Init_SPO2();        //初始化SPO2模块
}

/*********************************************************************************************************
* 函数名称：Proc500umTask
* 函数功能：0.5ms处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：
* 注    意：当亮灯时每1.5ms采一次样
*********************************************************************************************************/
static  void  Proc500usTask(void)
{
	if(Get500usFlag())      //判断0.5ms标志状态
  {
		Clr500usFlag();  //清除0.5ms标志
	}
}

/*********************************************************************************************************
* 函数名称：Proc2msTask
* 函数功能：2ms处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc2msTask(void)
{
	u16 adcData;      //队列数据
	u16  waveData2;     //波形数据
//	static u8 s_iCnt2 = 0;   //计数器
	
  if(Get2msFlag())  //判断2ms标志状态
  { 
//		if(s_iCnt2 >= 1)
//		{
			if (led_state == 0)
			{
				GPIO_ResetBits(IR_CS_PORT,IR_CS_PIN);
				GPIO_SetBits(RED_CS_PORT, RED_CS_PIN);
				led_state = 1;
			}
			else
			{
				GPIO_ResetBits(RED_CS_PORT, RED_CS_PIN);
				GPIO_SetBits(IR_CS_PORT, IR_CS_PIN);
				led_state = 0;
			}
//			s_iCnt2 = 0;
//		}
//		s_iCnt2++; 
    
//		if(ReadADCBuf(&adcData))  //从缓存队列中取出1个数据
//      {
//				waveData2 = (adcData * 127) / 4095;  //计算获取点的位置
  
				
				if (led_state == 1)
				{
					DACdate = GetIRDA();//红外光DA值
					DAC_SetChannel1Data(DAC_Align_12b_R, DACdate); 
//					if(ReadADCBuf(&adcData)) 
//					{
						waveData2 =s_arrADC1Data;;  //计算获取点的位置
				
//						waveData2 = notch_filter(waveData2);	//陷波滤波
//						waveData2 = IIRFilterECG(waveData2);	//IIR滤波
//						waveData2 = FIRFilterECG(waveData2);	//FIR滤波
						
						waveData2 = MovingAverageFilter_Pro(&IR_movingFilter, waveData2);   //移动平滑滤波
						IRwave = waveData2;
						waveData2 = FilterPro(&IR_HighPass, waveData2)+900;
						waveData2 = FIRFilterECG(waveData2);  //FIR滤波处理      
						IR_wavedata = waveData2;
//						IRwave = waveData2;
						min(IRwave,1);
						max(IRwave,1);
						iravg = GetIRavg();
						receive_cnt++;
//					}
				}
				else
				{
					DACdate = GetRedDA();//红外光DA值
					DAC_SetChannel1Data(DAC_Align_12b_R, DACdate); 
//					if(ReadADCBuf(&adcData)) 
//					{
						waveData2 =s_arrADC1Data;  //计算获取点的位置
						
//						waveData2 = notch_filter(waveData2);	//陷波滤波
//						waveData2 = IIRFilterECG(waveData2);	//IIR滤波
//						waveData2 = FIRFilterECG(waveData2);	//FIR滤波
						
						waveData2 = MovingAverageFilter_Pro(&RED_movingFilter, waveData2);   //移动平滑滤波
						REDwave = waveData2;

//        RedWave = FilterPro(&RED_LowPass, RedWave);                           //低通
						waveData2 = FilterPro(&RED_HighPass, waveData2)+900;
						waveData2 = FIRFilterECG(waveData2);  //FIR滤波处理    
						RED_wavedata = waveData2;
//						REDwave = waveData2;
						min(REDwave,0);
						max(REDwave,0);
						redavg = GetREDavg();
						receive_cnt++;
//					}
				}
//				heartrate = calc_heart_rate();	//计算心率
				detect_peak(RED_wavedata);	//检测波值
				if(receive_cnt >= 500)  //一屏数据有1024个点，每次接收到红光和红外光波形数据receive_cnt都加1，故receive_cnt = 2048才满一屏数据
				{
						receive_cnt = 0;
						pulseRate = calc_pulse_rate();
						Spo2 = calc_spo2();
		//				printf("%d\r\n",waveData2); 
										
		//      printf("[[1,%d]]\r\n", heartrate); //设置测量结果显示
						printf("[[1,%d]]\r\n", pulseRate); //设置测量结果显示
						printf("[[2,%d%%]]\r\n", Spo2); //设置测量结果显示
						RESETdata();
				}
				if (led_state == 1)
				{
					printf("%d,%d\r\n",IR_wavedata, RED_wavedata);
				}
//    if(ReadUART1(&uart1RecData, 1)) //读串口接收数据
//    {       
//      ProcHostCmd(uart1RecData);  //处理命令      
//    }

//        s_arrWaveData[s_iPointCnt] = waveData;  //存放到数组
//        s_iPointCnt++;  //波形数据包的点计数器加1操作

//        if(s_iPointCnt >= 5)  //接收到5个点
//        {
//          s_iPointCnt = 0;  //计数器清零
//          SendWaveToHost(s_arrWaveData);  //发送波形数据包
//        }
//    }   
		LEDFlicker(250);//调用闪烁函数     
    Clr2msFlag();   //清除2ms标志
  }
}

/*********************************************************************************************************
* 函数名称：Proc1SecTask
* 函数功能：1s处理任务 
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
static  void  Proc1SecTask(void)
{ 
  if(Get1SecFlag()) //判断1s标志状态
  {
    //printf("This is the first STM32F103 Project, by Zhangsan\r\n");
		FINGERtest(IRwave,REDwave,iravg,redavg);
    printf("[[5,%d]]\r\n", DACdate); 
    Clr1SecFlag();  //清除1s标志
  }    
}

/*********************************************************************************************************
* 函数名称：main
* 函数功能：主函数 
* 输入参数：void
* 输出参数：void
* 返 回 值：int
* 创建日期：2018年01月01日
* 注    意：
*********************************************************************************************************/
int main(void)
{ 
  InitSoftware();   //初始化软件相关函数
  InitHardware();   //初始化硬件相关函数
	RoughAdj();
//  printf("[[1,心率]]\r\n");    //设置参数名显示
//  printf("[[2,导联]]\r\n");    //设置参数名显示
	printf("[[1,脉率]]\r\n");    //设置参数名显示
	printf("[[2,血氧饱和度]]\r\n");    //设置参数名显示
	printf("[[3,调光状态]]\r\n");    //设置参数名显示
	printf("[[4,手指连接]]\r\n");    //设置参数名显示
	printf("[[5,DA]]\r\n");    //设置参数名显示
	printf("[[4,脱落]]\r\n"); 
  while(1)
  {
//		Proc500usTask();//0.5ms处理任务
    Proc2msTask();  //2ms处理任务
    Proc1SecTask(); //1s处理任务   
		
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7))
//			printf("[[2,脱落]]\r\n"); 
//    else
//      printf("[[2,连接]]\r\n"); 
  }
}
