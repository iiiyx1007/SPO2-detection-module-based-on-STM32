/*********************************************************************************************************
* 模块名称：SPO2.h
* 摘    要：血氧模块
* 当前版本：1.0.0
* 作    者：
* 完成日期：2025/7/31
* 内    容：
* 注    意：                                                                  
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
#include "Filiter.h"
#include "spo2check.h"
#include <stdio.h> 
#include "stm32f10x_tim.h"
#include "UART1.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define MIN_IR_INT_DA_VAL   100		//红外光强最小D/A值，红外光调光的起始值
#define MIN_RED_INT_DA_VAL  100		//红光光强最小D/A值，红光调光的起始值
#define MAX_IR_INT_DA_VAL   250		//红外光光强最大D/A值
#define MAX_RED_INT_DA_VAL  250 	//红光光强最大D/A值

#define MAX_CNT_ADJ_IR_DELAY  3	//调节红外光延时计数器最大值
#define MAX_CNT_ADJ_RED_DELAY 3 //调节红光延时计数器最大值

//红外光调光成功中线，第一次小于该值时将触发完成调红外光条件计数器，计数器大于或等于3将gRoughAdjIRFlag置1
#define IR_CENTRAL_LINE  2000			
//红光调光成功中线，第一次小于该值时将触发完成调红光条件计数器，计数器大于或等于3将gRoughAdjRedFlag置1
#define RED_CENTRAL_LINE 1700

#define	IR_OFF_MAX_VAL      80 						//红外光A/D值小于该值，表示手指探头脱落
#define RED_OFF_MAX_VAL  		80 						//红光A/D值小于该值，表示手指探头脱落
#define MAX_CNT_ASTABLE_FINGER 		 20			//手指不稳定计数器最大值，大于或等于该值时表示手指探头脱落
#define MAX_CNT_STABLE_FINGER  		20 			//手指稳定计数器最大值，大于或等于该值时表示手指探头接入
#define MAX_CNT_ROUGH_ADJ_IR_COND 	3	  	//完成粗调红外光条件计数器最大值
#define MAX_CNT_ROUGH_ADJ_RED_COND  3     // 完成调红光条件计数器最大值

#define RE_ADJ_IR_TOP_LINE   2500			//重调红外光上限A/D值，超过上限A/D值需要重新调光
#define RE_ADJ_IR_BOT_LINE   1500			//重调红外光下限A/D值，超过下限A/D值需要重新调光
#define RE_ADJ_RED_TOP_LINE  2200	  	//重调红光上限A/D值，超过上限A/D值需要重新调光
#define RE_ADJ_RED_BOT_LINE  1200	  	//重调红光下限A/D值，超过下限A/D值需要重新调光

#define IR_FINE_ADJ_OFFSET   300		//红外光细调上下偏移量
#define RED_FINE_ADJ_OFFSET  300		//红光细调上下偏移量#define MAX_CNT_ROUGH_ADJ_RED_COND 			3 //完成粗调红光条件计数器最大值

#define MOVING_FILTER_SIZE 10      //平滑滤波窗口
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
MovingAverageFilter IR_movingFilter;
MovingAverageFilter RED_movingFilter;

Filter              IR_HighPass;
Filter              RED_HighPass;
Filter              IR_LowPass;
Filter              RED_LowPass;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
u16 gIRADVal = 0;  //红外光AD值,初始为0
u16 gRedADVal = 0;  //红光AD值,初始为0

u16 gIRIntDAVal = MIN_IR_INT_DA_VAL;   //红外光光强DA值
u16 gRedIntDAVal = MIN_RED_INT_DA_VAL;  //红光光强DA值

int gIRADAvg;								//红外光波形数据平均值
int gRedADAvg;							//红光波形数据平均值

//状态/标志变量
int gRoughAdjIRFlag = 0;		//红外光调节完成标志，0未完成，1完成
int gRoughAdjRedFlag = 0;		//红光调节完成标志，0未完成，1完成
int gFirDetFinger = 0;			//重新调光后第一次检测到手指指标
int gFingerOnFlag = 0;				//手指探头实时连接状态，0——链接，1——脱离

//延时计数器
int gRoughAdjIRCondCnt  = 0;	//完成调红光外条件计数器
int gRoughAdjRedCondCnt = 0;	//完成调红光条件计数器

int gAstableFingerCnt = 0;	  //手指处于不稳定状态计数器
int gStableFingerCnt =  0;		//手指处于稳定状态计数器

int gAdjIRDelayCnt =  0;			//调节红外光延时计数器
int gAdjRedDelayCnt = 0;			//调节红光延时计数器

int gIROutRngCnt =  0;					//红外光超出范围计数器
int gRedOutRngCnt = 0;					//红光超出范围计数器

float arr_RedFilter[1500] = {0}; //红光滤波数组
float arr_IrFilter[1500] = {0};  //红外光滤波数组
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static u8 CheckRoughAdj(void);
static void ConfigSPO2GPIO(void);
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigSPO2GPIO
* 函数功能：配置使能血氧测量和红光和红外光控制的GPIO
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2025/7/31
* 注    意：
*********************************************************************************************************/
//对原始数据进行打包
//static void Pack(const u8 in_pack[10],u8 out_pack[10]);
//static void Pack(const u8 in_pack[10],u8 out_pack[10])
//{
//	u8 checkSum;
//	u8 dataHead;
//	int i;
//	
//	//复制输入到输出，保证原始数据不变
//	for(i = 0;i < 10;i++)
//	{
//		out_pack[i] = in_pack[i];
//	}
//	
//	checkSum = out_pack[0];	//取出ID作为校验和初始值
//	dataHead = 0;	//数据头清零
//	
//	//逆序处理索引8到2
//	for(i = 8;i >=2; i--)
//	{
//		dataHead = (u8)(dataHead << 1);
//		
//		out_pack[i] = out_pack[i-1]|0x80;//最高位置一，其他位保持不变
//		
//		checkSum = (u8)(checkSum + out_pack[i]);
//		
//		dataHead |= ((out_pack[i-1] & 0x80) >> 7);
//	}
//	
//	out_pack[1] = dataHead | 0x80;
//	checkSum = (u8)(checkSum + out_pack[1]);
//	
//	out_pack[9] = checkSum | 0x80;	
//}

static u8 CheckRoughAdj(void)
{
  static u8 finRoughAdj;  //粗调完成标志
  if (gRoughAdjIRFlag == 1 && gRoughAdjRedFlag == 1) // 判断粗调是否完成
    finRoughAdj = 1; // 粗调完成
  else
    finRoughAdj = 0; // 粗调未完成 
  return finRoughAdj;
}

static void ConfigSPO2GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使能RCC相关时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//设置上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//设置上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOB, GPIO_Pin_9); 
}

void Init_SPO2(void)
{
	ConfigSPO2GPIO();
	MovingAverage_Init(&IR_movingFilter, arr_IrFilter, MOVING_FILTER_SIZE);
  MovingAverage_Init(&RED_movingFilter, arr_RedFilter, MOVING_FILTER_SIZE);
  HighPassFilterInit(&IR_HighPass,0.5, 250, 0.707);
  HighPassFilterInit(&RED_HighPass,0.5, 250, 0.707);
  LowPassFilterInit(&IR_LowPass,20, 600);
  LowPassFilterInit(&RED_LowPass,20, 600);
}

////调参
//void SendAdjCmd(u16 spo_ir,u16 spo_red)
//{
//	u8 raw_pack[10] = {0};
//	u8 packed_data[10];
//	
//	raw_pack[0] = 51;
//	raw_pack[1] = 129;
//	raw_pack[2] = (u8)((spo_ir >> 8) & 0xFF);
//	raw_pack[3] = (u8)(spo_ir & 0xFF);
//	raw_pack[4] = (u8)((spo_red >> 8) & 0xFF);
//	raw_pack[5] = (u8)(spo_red & 0xFF);
//	
//	Pack(raw_pack,packed_data);
//	
////	WriteUART1(packed_data,10);
//}


////复位调光参数，准备重新调光
//void ResetAdj(void)
//{
//	SendAdjCmd(MIN_IR_INT_DA_VAL,MIN_RED_INT_DA_VAL);
//	SendAdjCmd(MIN_IR_INT_DA_VAL,MIN_RED_INT_DA_VAL);
//}


//复位调光参数，准备重新调光
void ResetAdj(void)
{
  gRoughAdjIRFlag = 0;   //红外光粗调完成标志，0-未完成，1-完成
  gRoughAdjRedFlag = 0;  //红光粗调完成标志，0-未完成，1-完成
  gRoughAdjIRCondCnt = 0;  //完成粗调红外光条件计数器
  gRoughAdjRedCondCnt = 0;  //完成粗调红光条件计数器
  gFirDetFinger = 0; // 复位重新调光后第一次检测到手指标志为0
  gAstableFingerCnt = 0; // 复位值为0
  gStableFingerCnt = 0; // 复位值为0
  gIROutRngCnt = 0; // 复位红外光超出范围计数器
  gRedOutRngCnt = 0; // 复位红光超出范围计数器

  gIRIntDAVal = MIN_IR_INT_DA_VAL;   //红外光光强DA值
  gRedIntDAVal = MIN_RED_INT_DA_VAL;  //红光光强DA值
  printf("[[6,重新调光]]\r\n"); 

}

//粗调红外光强和红光光强
void RoughAdj(void)
{
	if(gFirDetFinger == 1)//如果检测到手指接入
	{
		if(gIRADVal < IR_OFF_MAX_VAL||gRedADVal < RED_OFF_MAX_VAL )//若检测到手指探头脱落
		{
			gAstableFingerCnt++;//手指处于不稳定状态计数器加1
			if(gAstableFingerCnt >= MAX_CNT_ASTABLE_FINGER)//连续20次检测到手指探头脱落，才表示手指探头确实脱落
			{
				ResetAdj();					//复位调光参数，准备重新调光
				gFingerOnFlag = 0;	//手指探头脱落
			}
		}
		else
		{
			gAstableFingerCnt = 0;		//手指处于不稳定状态计数器清零
			gFingerOnFlag = 1;				//手指连接
			if(gStableFingerCnt < MAX_CNT_STABLE_FINGER)
			{
				gStableFingerCnt ++;		//手指处于稳定状态定时器加1
			}
			else//连续20次检测到手指探头接入，才表示手指确实接入
			{
//				gFingerOnFlag = 1;				//手指连接
				//粗调红外光强
				if(gRoughAdjIRFlag == 0)//判断红外光粗调完成情况
				{
					if(gIRADVal < IR_CENTRAL_LINE)//粗调到位
					{
						gRoughAdjIRCondCnt++;//计数器加1
						if(gRoughAdjIRCondCnt >= MAX_CNT_ROUGH_ADJ_IR_COND)//连续三次符合要求，才表示粗调完成
						{
							gRoughAdjIRFlag = 1;//表示红外光粗调完成
						}
					}
					else//红外光A/D值大于或等于中线
					{
						gAdjIRDelayCnt ++;		//延时计数器+1
						if(gAdjIRDelayCnt >= MAX_CNT_ADJ_IR_DELAY)//如果延时完成
						{
							gAdjIRDelayCnt = 0;//粗调红外光延时计数器清零
							if(gIRIntDAVal < MAX_IR_INT_DA_VAL)//如果红外光光强D/A值小于最大值
							{
								gIRIntDAVal = gIRIntDAVal + 2;//则+2对红外光进行粗调
								//SendAdjCmd(gIRIntDAVal,gRedIntDAVal);//向从机发送调光指令
								DAC_SetChannel1Data(DAC_Align_12b_R, gIRIntDAVal);    // 增强红外光光强 
							}
							else//如果红外光光强D/A值大于或等于最大值
							{
								ResetAdj();//则复位调光参数
							}
							gRoughAdjIRCondCnt = 0;//完成粗调红外光，条件计数器清零
						}
					}
				}
				
				//粗调红光光强
				if(gRoughAdjRedFlag == 0)//判断红光粗调完成情况
				{
					if(gRedADVal < RED_CENTRAL_LINE)//粗调到位
					{
						gRoughAdjRedCondCnt++;//计数器加1
						if(gRoughAdjRedCondCnt >= MAX_CNT_ROUGH_ADJ_RED_COND)//连续三次符合要求，才表示粗调完成
						{
							gRoughAdjRedFlag = 1;//表示红光粗调完成
						}
					}
					else//红外光A/D值大于或等于中线
					{
						gAdjRedDelayCnt ++;		//延时计数器+1
						if(gAdjRedDelayCnt >= MAX_CNT_ADJ_RED_DELAY)//如果延时完成
						{
							gAdjRedDelayCnt = 0;//粗调红光延时计数器清零
							if(gRedIntDAVal < MAX_RED_INT_DA_VAL)//如果红光光强D/A值小于最大值
							{
								gRedIntDAVal = gRedIntDAVal + 2;//则+2对红光进行粗调
								//SendAdjCmd(gIRIntDAVal,gRedIntDAVal);//向从机发送调光指令
								DAC_SetChannel1Data(DAC_Align_12b_R, gRedIntDAVal);    // 增强红光光强 
							}
							else//如果红光光强D/A值大于或等于最大值
							{
								ResetAdj();//则复位调光参数
							}
							gRoughAdjRedCondCnt = 0;//完成粗调红光，条件计数器清零
						}
					}
				}
			}
		}
	}
	else//未检测到手指探头接入
	{
		if(gIRADVal > IR_OFF_MAX_VAL || gRedADVal > RED_OFF_MAX_VAL)//检测到手指接入
		{
//			printf("%d",gIRADVal);
			gStableFingerCnt = 0;	//将手指处于稳定状态计数器清零，准备计数	
			gFirDetFinger = 1;		//将该值置为1，表示已经检测到手指
		}
		gIRIntDAVal = MIN_IR_INT_DA_VAL;	//将红外光强D/A值设定为初值，准备调光
		gRedIntDAVal = MIN_RED_INT_DA_VAL;//将红光光强D/A值设定为初值，准备调光
	}
}

//判断是否需要重新调光
void JudgeReAdj(void)
{
	if(gRoughAdjIRFlag == 1)		//红外光粗调完成
	{
		if(gIRADAvg > RE_ADJ_IR_TOP_LINE || gIRADAvg < RE_ADJ_IR_BOT_LINE)//A/D值不再范围内
		{
			gIROutRngCnt++;					//计数器+1
			if(gIROutRngCnt >= 3)		//连续3次符合要求（手指动作比较大）
			{
				gRoughAdjIRFlag = 0;	//将标志清零，准备重新调光
				gIROutRngCnt = 0;			//计数器清零
			}
		}
		else
		{
			gIROutRngCnt = 0;				//计数器清零
		}
	}
	else
	{
		ResetAdj();				//复位调光参数，准备重新调光计数器清零
		gIROutRngCnt = 0;	//计数器清零
	}
	
	if(gRoughAdjRedFlag == 1)//红光粗调完成
	{
		if(gRedADAvg > RE_ADJ_RED_TOP_LINE || gRedADAvg < RE_ADJ_RED_BOT_LINE)
		{
			gRedOutRngCnt++;		  	//计数器+1
			if(gRedOutRngCnt >= 3)	//连续三次符合要求
			{
				gRoughAdjRedFlag = 0; //将标志位清零，准备重新调光
				gRedOutRngCnt = 0;		//计数器清零
			}
		}
		else
		{
			gRedOutRngCnt = 0;			//计数器清零
		}
	}
	else
	{
		ResetAdj();									//复位调光参数，准备重新调光
		gRedOutRngCnt = 0;					//计数器清零
	}
}

//细调红外光和红光光强
void FineAdj(void)
{
	//粗调完成才有可能进行微调
	if(gRoughAdjIRFlag == 1 && gRoughAdjRedFlag == 1)
	{
		//在微调范围内，调节红外光光强
		if(gIRADAvg > (IR_CENTRAL_LINE + IR_FINE_ADJ_OFFSET))//光过弱，但在微调范围内
		{
			gIRIntDAVal++;																		 //递增1进行微调，增强红外光光强
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);							 //向从机发送调光指令
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);							 //多发一次，确保成功发送
			 DAC_SetChannel1Data(DAC_Align_12b_R, gIRIntDAVal);    // 增强红外光光强 
			 printf("[[6,正在微调]]\r\n"); 
		}
		else if(gIRADAvg < (IR_CENTRAL_LINE - IR_FINE_ADJ_OFFSET))//光过强，但在微调范围内
		{
			gIRIntDAVal--;																		//递减1进行微调，增强红外光光强
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);							//向从机发送调光指令
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);						  //多发一次，确保成功发送
			DAC_SetChannel1Data(DAC_Align_12b_R, gIRIntDAVal);    // 增强红外光光强 
			printf("[[6,正在微调]]\r\n"); 
		}
		else
		{
 			printf("[[6,光强适中]]\r\n");
		}
		//在微调范围内，调节红光光强
		if(gRedADAvg > (RED_CENTRAL_LINE + RED_FINE_ADJ_OFFSET))
		{
			gRedIntDAVal++;
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);
      DAC_SetChannel1Data(DAC_Align_12b_R, gRedIntDAVal);    // 增强红光光强
			printf("[[6,正在微调]]\r\n"); 
		}
		else if(gRedADAvg < (RED_CENTRAL_LINE - RED_FINE_ADJ_OFFSET))
		{
			gRedIntDAVal--;
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);
//			SendAdjCmd(gIRIntDAVal,gRedIntDAVal);
			DAC_SetChannel1Data(DAC_Align_12b_R, gRedIntDAVal);    // 增强红光光强 
			printf("[[6,正在微调]]\r\n"); 
		}
		else
		{
			printf("[[6,光强适中]]\r\n");
		}
	}
}

void FINGERtest(int irdata,int reddata,int iravg,int redavg)
{
	gIRADAvg = iravg;					//红外光波形数据平均值
	gRedADAvg = redavg;				//红光波形数据平均值
	gIRADVal = irdata;				//红外光AD值
	gRedADVal = reddata;			//红光AD值
	
	if((gIRADVal < IR_OFF_MAX_VAL) || (gRedADVal < RED_OFF_MAX_VAL)) 
	{
		gAstableFingerCnt++;
		if(gAstableFingerCnt >= MAX_CNT_ASTABLE_FINGER)
		{
			ResetAdj();
			gFingerOnFlag = 0;
			printf("[[4,脱落]]\r\n"); 
		}
	}
	if(CheckRoughAdj() == 1)
	{
		JudgeReAdj(); // 判断是否需要重新调光，如果需要则进行重新调光
    FineAdj(); // 判断是否需要微调，如果需要则进行微调
		printf("[[3,粗调完成]]\r\n"); 
	}
	else
	{
		RoughAdj(); 
		if (gFirDetFinger == 1) // 重新调光后，如果第一次检测到手指
        printf("[[3,正在调光]]\r\n");  
    else // 重新调光后，如果还未检测到手指
        printf("[[3,---]]\r\n");  
	}
//	RoughAdj();//粗调红外光和红光光强
//	if(gRoughAdjIRFlag == 1 && gRoughAdjRedFlag == 1)
//	{
//		JudgeReAdj();
//		FineAdj();
//		printf("[[6,粗调完成]]\r\n");
//	}
//	else//若粗调未完成
//	{
//		if(gFirDetFinger == 1)
//		{
//			printf("[[6,正在调光]]\r\n");
//		}
//		else
//		{
//			printf("[[6,---]]\r\n");
//		}
//	}
		if(gFingerOnFlag == 1)
		{
		printf("[[4,连接]]\r\n");
		}
		else
		{
		printf("[[4,脱落]]\r\n"); 
		}
		printf("[[5,连接]]\r\n");
		printf("[[5,连接]]\r\n");
}

u16 GetIRDA(void)
{
	return gIRIntDAVal;
}

u16 GetRedDA(void)
{
	return gRedIntDAVal;
}


u16 maxIRData, minIRData = 4095, maxRedData, minRedData = 4095;  //红外光和红光最大
void max(float filtered_data,int tag)
{
	if(tag ==1)
	{
		if(filtered_data > maxIRData)
		{
			maxIRData = filtered_data;
		}
	}
	else
	{
		if(filtered_data > maxRedData)
		{
			maxRedData = filtered_data;
		}
	}
}

void min(float filtered_data,int tag)
{
	if(tag ==1)
	{
		if(filtered_data < minIRData)
		{
			minIRData = filtered_data;
		}
	}
	else
	{
		if(filtered_data < minRedData)
		{
			minRedData = filtered_data;
		}
	}
}


short calc_spo2(void)
{
	int irADRng;
	int redADRng;
	unsigned int RR_TABLE[11] = {580,620,650,670,700,730,760,780,810,840,880};
//	unsigned int RR_TABLE[11] = {3000, 2700, 2500, 2400, 2300, 2200, 2100, 2000, 1900, 1800};//R值表

	short o2;
	float rVal;
	int index;
	
	irADRng = maxIRData - minIRData;
	redADRng = maxRedData - minRedData;
	if(irADRng > 0)
	{
		rVal = redADRng *1000 / irADRng;
	}
	
	index = 1;
	
	while(rVal >= RR_TABLE[index]&&index < 11)
	{
		index++;
	}
	
	o2 = 101 - index;
	
	if(o2 == 100)
	{
		o2 = 99;
	}
	
	return o2;
}

int GetIRavg(void)
{
	return (maxIRData + minIRData)/2;
}

int GetREDavg(void)
{
	return (maxRedData + minRedData)/2;
}

void RESETdata(void)
{
	maxIRData = 0;
	minIRData = 4095;
	maxRedData = 0;
	minRedData = 4095; 
}



