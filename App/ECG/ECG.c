/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "ECG.h"
#include <math.h>
#include "stm32f10x_tim.h"
#include "ADC.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define PI 3.1415926535f
#define FS 500.0f			//采样频率

//陷波器常量
#define FC 50.0f			//陷波频率，截止频率
#define ALPHA -1.99f	//#define ALPHA （-1 * 2） * conf（2*PI*FC/FS）
#define BETA 0.96f

//IIR滤波器常量(通过Matlab计算得到)
#define GAIN 0.0048f

//FIR滤波参数：抽头数 = 5，阶数 = 4
#define FIR_TAPS 5		//(sizeof（fir_coeff） / sizeof（fir_coeff[0]）)

////心率计算
//#define MIN_HEART_RATE 30
//#define MAX_HEART_RATE 240

//血氧计算
#define MIN_PULSE_RATE  20
#define MAX_PULSE_RATE  120
//#define MIN_HEART_RATE 0       // 心率最小值（无效值判断）
//#define MAX_HEART_RATE 330     // 心率最大值（无效值判断）
#define PEAK_DIST_MIN  300     // 最小峰值间隔（单位：采样点，对应500ms@500Hz，避免误检）

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
/*1、陷波器系数计算*/
static float notch_b[3] ={1,ALPHA,1};
static float notch_a[3] = {1,(ALPHA) * BETA,BETA*BETA};

static float x_buf[2] = {0};
static float y_buf[2] = {0};
//IIR滤波器系数
//二阶环节系数（MATLAB生成，每行取（b0,b1,b2,a1,a2），相对于MATLAB步骤四的[b,a]
const float iir_sos[2][5] = {
	{1.0000f, 2.0000f, 1.0000f, -1.0486f, 0.2961f},
	{1.0000f, 2.0000f, 1.0000f, -1.3209f, 0.6327f}
};

//缓存数组:保存每个二阶环节的历史输入/输出(两个环节*两个缓存）
static float iir_x_buf[2][2] = {{0},{0}};
static float iir_y_buf[2][2] = {{0},{0}};

//FIR滤波器参数,相对于MATLAB步骤五的b
static const float fir_coeff[] = {
	0.0246f,	//b[0]
	0.2344f,	//b[1]
	0.4821f,	//b[2]
	0.2344f,	//b[3]
	0.0246f	  //b[4]
};

//输入数据缓存（滑动窗口，保存最近FIR_TAPS个采样值）
static float fir_buffer[FIR_TAPS] = {0.0f};

//心率计算
unsigned int peak_index[10]; 	//缓存最近10个峰值的采样点索引（用于计算间隔）
unsigned char peak_count = 0; //已检测到的峰值数量

//血氧饱和度计算
//unsigned int peak_IR_max[3],peak_RED_max[3],peak_IR_min[3],peak_RED_min[3];
unsigned int ir_count_max,red_count_max,ir_count_min,red_count_min;
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static unsigned int median(unsigned int arr[], unsigned int len); // 辅助函数：计算数组中值（抗干扰）
static float biquad_filter(float x, unsigned char section); // 二阶IIR滤波单元
static void ConfigLEADGPIO(void);   //配置导联连接/脱落的GPIO

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//辅助函数；计算数组中值（抗干扰）
static unsigned int median(unsigned int arr[],unsigned int len)
{
	unsigned char i,j;
	unsigned int temp;
	
	//冒泡法
	for(i = 0;i < len;i++)
	{
		for(j = i + 1;j < len;j++)
		{
			if(arr[i] > arr[j])
			{
				temp = arr[i];
				arr[i] = arr[j];
				arr[j] = temp;
			}
		}
	}
	return arr[len / 2];
}

/**
 * 二阶IIR滤波单元
 * @param x：当前输入数据
 * @param section：二阶环节索引（0或1）
 * @return 滤波后的数据
 */
static float biquad_filter(float x,unsigned char section)
{
	//提取当前环节的系数
	float b0 = iir_sos[section][0];
	float b1 = iir_sos[section][1];
	float b2 = iir_sos[section][2];
	float a1 = iir_sos[section][3];
	float a2 = iir_sos[section][4];
	
	//计算当前输出（差分方程）
	static float y;
	y = b0 * x 
			+ b1 * iir_x_buf[section][0] 
			+ b2 * iir_x_buf[section][1] 
			- a1 * iir_y_buf[section][0]
			- a2 * iir_y_buf[section][1];
	
  // 更新缓存（左移操作，保存最新的历史数据）
  iir_x_buf[section][1] = iir_x_buf[section][0];
  iir_x_buf[section][0] = x;
  iir_y_buf[section][1] = iir_y_buf[section][0];
  iir_y_buf[section][0] = y;
  
  return y;
}

static void ConfigLEADGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//使能RCC相关时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//设置上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}
	
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
//初始化ECG模块
void Init_ECG(void)
{
	ConfigLEADGPIO(); //配置导联连接/脱落的GPIO
}

//陷波滤波函数（x为输入，y为输出，需保存前2次输入输出）
float notch_filter(float x)
{
	static float y;
	y = (notch_b[0] * x + notch_b[1] * x_buf[0] + notch_b[2] * x_buf[1]
			- notch_a[1] * y_buf[0] - notch_a[2] * y_buf[1]) / notch_a[0];
	//更新缓存
  x_buf[1] = x_buf[0]; x_buf[0] = x;
  y_buf[1] = y_buf[0]; y_buf[0] = y;
  return y;
}

/**
 * IIR滤波主函数（等效于MATLAB的filter(b,a,dataIn)）
 * @param dataIn：输入的原始采样数据
 * @return 滤波后的数据
 */
float IIRFilterECG(float x) 
{
  static float y;
  
  // 级联第一个二阶二阶环节滤波
  y = biquad_filter(x, 0);
  
  // 第二个二阶环节滤波（级联处理）
  y = biquad_filter(y, 1);
  
  return y * GAIN;
}

//3、FIR滤波部分
/**
 * FIR滤波函数（等效于MATLAB的filter(b, 1, dataIn)）
 * @param dataIn：当前输入的采样数据（单次值）
 * @return 滤波后的数据
 */
float FIRFilterECG(float x)
{
	static unsigned char i;
	float y = 0.0f;
	
	//1.更新滑动窗口：数据左移，新数据存入最前端
	for(i = FIR_TAPS - 1;i > 0;i--)
	{
		fir_buffer[i] = fir_buffer[i - 1];
	}
	fir_buffer[0] = x;//存入最新采样值
	
	//2.卷积计算：系数与缓存数据相乘累加
	for(i = 0;i < FIR_TAPS;i++)
	{
		y += fir_coeff[i] * fir_buffer[i];
	}
	
	return y;
}

//4、计算心率
// 峰值检测状态变量
unsigned char peak_flag = 0;       // 峰值检测标记（1=检测到峰值）
float last_val = 0;          // 上一时刻的滤波后数据
//float threshold = 70;         // 心率动态阈值（自适应信号幅度）
float threshold = 910;         // 血氧动态阈值（自适应信号幅度）

void detect_peak(float filtered_data) 
{
  unsigned int current_index;
  static unsigned int last_index = 0;
  // 1. 动态更新阈值（避免固定阈值受信号幅度变化影响）
//  threshold = 0.3f * filtered_data + 0.7f * threshold; // 低通滤波平滑阈值

  // 2. 峰值判断：当前值 > 上一值 且 当前值 > 阈值 且 过零点后上升沿
  if (filtered_data > last_val && filtered_data > threshold) 
  {
    peak_flag = 1; // 标记可能的峰值
  } 
  else if (peak_flag && filtered_data < last_val) 
  {
    // 确认峰值：从上升转为下降时，记录当前采样点索引
    current_index = Get_ADC_Num(); // 获取当前采样点数（单位：采样点）
    if(current_index - last_index >= PEAK_DIST_MIN)
    {
      peak_index[peak_count % 10] = current_index; // 循环缓存最近10个峰值
  //    peak_count = (peak_count + 1) % 10;
      peak_count++;     
      last_index = current_index;
    }
//    peak_index[peak_count % 10] = current_index; // 循环缓存最近10个峰值
////    peak_count = (peak_count + 1) % 10;
//    peak_count++;
    peak_flag = 0; // 重置标记
    
  }
  last_val = filtered_data; // 更新上一时刻值
}


//unsigned int current_MAXwave;
//unsigned int median_interval1_red,median_interval1_ir;
//void max(float filtered_data,int tag)
//{
//	unsigned int peak_IR_max[3] = {0},peak_RED_max[3] = {0};
//	int num2;
//	if(filtered_data > last_val && filtered_data > threshhiold)
//	{
//		peak_flag_max = 1;
//	}
//	else if(peak_flag_max && filtered_data < last_val)
//	{
//		peak_flag_max = 0;
//		current_MAXwave = last_val;			
//		if(tag == 1)
//		{
//			if(ir_count_min >= 2)
//			{
//				num2 = 2;
//			}
//			if(ir_count_min < 2) num2 = ir_count_min;
//			peak_IR_max[ir_count_max % 3] = current_MAXwave;	
//			median_interval1_ir =  median(peak_IR_max,num2);
//			ir_count_max++;
//		}
//		else if(tag == 0)
//		{
//			if(ir_count_min >= 2)
//			{
//				num2 = 2;
//			}
//			if(ir_count_min < 2) num2 = ir_count_min;
//			peak_RED_max[red_count_max % 3] = current_MAXwave;		
//			median_interval1_red = median(peak_RED_max,num2);
//			red_count_max++;
//		}
//	}
//}

//unsigned int current_MINwave;
//unsigned int median_interval2_red,median_interval2_ir;
//void min(float filtered_data,int tag)
//{
//	unsigned int peak_IR_min[5] = {0},peak_RED_min[5] = {0};
//	int num1;
//	if(filtered_data < last_val && filtered_data < threshhiold)
//	{
//		peak_flag_min = 1;
//	}
//	else if(peak_flag_min && filtered_data > last_val)
//	{
//		peak_flag_min = 0;
//		current_MINwave = last_val;		
//		if(tag == 1)
//		{
//			if(ir_count_min >= 2)
//			{
//				num1 = 2;
//			}
//			if(ir_count_min < 2) num1 = ir_count_min;
//			peak_IR_min[ir_count_min % 5] = current_MINwave;			
//			median_interval2_ir =  median(peak_IR_min,num1);
//			ir_count_min++;
//		}
//		else if(tag == 0)
//		{
//			if(ir_count_min >= 2)
//			{
//				num1 = 2;
//			}
//			if(ir_count_min < 2) num1 = ir_count_min;
//			peak_RED_min[red_count_min % 5] = current_MINwave;			
//			median_interval2_red = median(peak_RED_min,num1);
//			red_count_min++;
//		}

//	}
//}

short calc_pulse_rate(void)
{
  unsigned int intervals[9] = {0};  //波峰间距
  unsigned int median_interval;  //9个波峰间隔的中间值
  static unsigned char first = 1; //第一次0~255,因为peak_count溢出后又会从0开始，为了防止错误判断小于两个峰值而退出函数
  unsigned char i;
  short hr;  //心率
  
  if(first && peak_count > 9)
    first = 0;
  if (peak_count < 2 && first) return 0; // 至少需要2个峰值才能计算间隔

  // 1. 计算最近N个峰值间隔（单位：采样点）
  for (i = 0; i < peak_count - 1 && i < 9; i++) 
  {
    intervals[i] = peak_index[i + 1] - peak_index[i];
  }

  // 2. 取中值（抗干扰，避免单次误检影响）
  median_interval = median(intervals, ((first && peak_count < 9) ? peak_count - 1 : 9));

  // 3. 转换为心率：心率 = 60 / (间隔时间)，间隔时间 = 间隔点数 / 采样频率
//  hr = 60*500 / median_interval;  //心率，采样率500Hz
  hr = 60*1000 / median_interval;  //血氧脉率，采样率250Hz

  // 4. 异常值处理（超出合理范围则视为无效）
//  if (hr < MIN_PULSE_RATE || hr > MAX_PULSE_RATE)  
////  if (hr > MAX_HEART_RATE) //unsigned short变量本身就不小于0，不必再与MIN_HEART_RATE进行比较
//  {
//    return -1; // 无效值标记
//  }
  return hr;
}


//short calc_heart_rate(void)
//{
//	unsigned int intervals[9] = {0};
//	unsigned int median_interval;
//	static unsigned char first = 1;
//	unsigned char i;
//	short hr; //心率

//	if(first && peak_count > 9)
//	{
//		first = 0;
//	}
//	if(peak_count < 2 && first ) return 0;
//// 1. 计算最近N个峰值间隔（单位：采样点）
//	for(i = 0; i < peak_count - 1 && i < 9; i++)
//	{
//		intervals[i] = peak_index[i + 1] - peak_index[i];
//	}
//	//取中值
//	median_interval = median(intervals,((first && peak_count < 9) ? peak_count - 1:9));
//	
//	//转换为心率
//	hr = 60 * FS / median_interval;
//	
//	// 4. 异常值处理（超出合理范围则视为无效）
//  if (hr < MIN_HEART_RATE || hr > MAX_HEART_RATE)  //unsigned short变量本身就不小于0，不必再与MIN_HEART_RATE进行比较
//  {
//    return -1; // 无效值标记
//  }
//	//  if (hr > MAX_HEART_RATE)
//  return hr;
//}

