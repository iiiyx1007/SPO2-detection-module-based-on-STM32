/*********************************************************************************************************
* 模块名称：Filiter.c
* 摘    要：Filter模块
* 当前版本：1.0.0
* 作    者：
* 完成日期：
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
#include "stm32f10x_conf.h"
#include "math.h"
#include "UART1.h"
/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define PI 3.141592653
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/


/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/

/*********************************************************************************************************

*********************************************************************************************************/


/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：NotchFilterInit
* 函数功能：初始化陷波器
* 输入参数：Filter *filter, f0-陷波频率, Q-品质因数, fs-采样频率
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
void NotchFilterInit(Filter *filter, float f0, float Q, float fs)
{
  //计算角频率和双线性变换参数
  //float w0 = 2 * PI * f0;                   //中心角频率
  float T = 1.0 / fs;                       //采样周期
  //float wa = (2.0f / T) * tanf(w0 * T / 2.0f);      //预畸变修正后的频率
  
  //传递函数系数
  float alpha = -2 * cos(2 * PI * f0 * T);
  float beta = 0.96;
  
  //双线性变换后的离散系数
  filter->b0 = 1.0f;
  filter->b1 = alpha;
  filter->b2 = 1.0f;
  filter->a0 = 1;
  filter->a1 = alpha * beta;
  filter->a2 = beta * beta;
  //printf("b0:%0.4f b1:%0.4f b2:%0.4f\n",filter->b0,filter->b1,filter->b2);
  //printf("a1:%0.4f a2:%0.4f\n",filter->a1,filter->a2);
  
  
  //初始化状态变量
  filter->x_perv[0] = 0;
  filter->x_perv[1] = 0;
  filter->x_perv[2] = 0;
  filter->y_perv[0] = 0;
  filter->y_perv[1] = 0;
  filter->y_perv[2] = 0;
}

/*********************************************************************************************************
* 函数名称：HighPassFilterInit
* 函数功能：初始化高通滤波器
* 输入参数：*filter-结构体 fc-截止频率 fs-采样频率 Q-品质因数
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：if(fc >= fs/2) fc = fs/4;  // 截止频率不能超过Nyquist频率
*********************************************************************************************************/
  void HighPassFilterInit(Filter *filter, float fc, float fs, float Q)
{
    u8 i = 0;
    // 2. 预扭曲频率计算(双线性变换)
    float omega = 2 * PI * fc / fs;
    float alpha = sin(omega) / (2 * Q);
    
    // 3. 模拟滤波器系数计算(二阶巴特沃斯高通)
    float b0 = (1 + cos(omega)) / 2;
    float b1 = -(1 + cos(omega));
    float b2 = b0;
    float a0 = 1 + alpha;
    float a1 = -2 * cos(omega);
    float a2 = 1 - alpha;

    // 4. 归一化数字滤波器系数
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
    
//  printf("b0:%0.4f b1:%0.4f b2:%0.4f\n",filter->b0,filter->b1,filter->b2);
//  printf("a1:%0.4f a2:%0.4f\n",filter->a1,filter->a2);
  
  //初始化状态变量
  filter->x_perv[0] = 0;
  filter->x_perv[1] = 0;
  filter->x_perv[2] = 0;
  filter->y_perv[0] = 0;
  filter->y_perv[1] = 0;
  filter->y_perv[2] = 0;
 }
/*********************************************************************************************************
* 函数名称：BandPassFilterInit
* 函数功能：初始化带通滤波器
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
void BandPassFilterInit(Filter *filter, float flow, float fhigh, float fs)
{
  float T = 1.0f / fs;
  float w0 = 2 * PI * sqrtf(flow * fhigh);
  float Q = w0 / (2 * PI * (fhigh - flow));
  float sqrt2 = sqrtf(2.0f);
  
  float w_low = 2 * PI * flow;
  float w_high = 2 * PI * fhigh;
  float w_low_prime = (2.0f / T) * tanf(w_low * T / 2.0f);
  float w_high_prime = (2.0f / T) * tanf(w_high * T / 2.0f);
  float w0_prime = sqrtf(w_low_prime * w_high_prime);
  
  float denominator = w0_prime * w0_prime + (w0_prime / Q) * w0_prime + w0_prime * w0_prime; 
  float beta = 1.0f / denominator;
  
  filter->b0 = (w0_prime / Q) * beta;
  filter->b1 = 0.0f;
  filter->b2 = -filter->b0;
  filter->a1 = 2 * beta * (1 - w0_prime * w0_prime);
  filter->a2 = beta * (1 - (w0_prime / Q) + w0_prime * w0_prime);
  
  //初始化状态变量
  filter->x_perv[0] = 0;
  filter->x_perv[1] = 0;
  filter->x_perv[2] = 0;
  filter->y_perv[0] = 0;
  filter->y_perv[1] = 0;
  filter->y_perv[2] = 0;
}

/*********************************************************************************************************
* 函数名称：LowPassFilterInit
* 函数功能：初始化低通滤波器-IIR
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：if(fc >= fs/2) fc = fs/2 - 1;  // 截止频率不能超过Nyquist频率
*********************************************************************************************************/
void LowPassFilterInit(Filter *filter, float fc, float fs)
{ 
    // 1. 预扭曲频率（双线性变换补偿）
    float omega = PI * fc;
    float T = 1.0f / fs;
    float omega_tan = tanf(omega * T);
    
    // 2. 计算模拟滤波器系数（二阶巴特沃斯）
    float K = omega_tan * omega_tan;
    float sqrt2 = sqrtf(2.0f);
    float denominator = 1.0f + sqrt2 * omega_tan + K;

    // 3. 双线性变换得到数字滤波器系数
    filter->b0 = K / denominator;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;
    
    filter->a0 = 1.0f;            // 规范化系数
    filter->a1 = 2.0f * (K - 1) / denominator;
    filter->a2 = (1 - sqrt2*omega_tan + K) / denominator;
    
//  printf("b0:%0.4f b1:%0.4f b2:%0.4f\n",filter->b0,filter->b1,filter->b2);
//  printf("a1:%0.4f a2:%0.4f\n",filter->a1,filter->a2);
    // 4. 初始化状态变量
    filter->x_perv[0] = filter->x_perv[1] = filter->x_perv[2] = 0.0f;
    filter->y_perv[0] = filter->y_perv[1] = filter->y_perv[2] = 0.0f;
}
/*********************************************************************************************************
* 函数名称：FilterPro
* 函数功能：处理单个采样点
* 输入参数：*filter input-待处理采样点
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
float FilterPro(Filter *filter, float input)
{
  float output = 0;
  //输入历史更新
  filter->x_perv[2] = filter->x_perv[1];
  filter->x_perv[1] = filter->x_perv[0];
  filter->x_perv[0] = input;
  
//  printf("HPF-input:%0.1f\n",input);
  //差分方程计算
  output = (filter->b0*filter->x_perv[0]) + (filter->b1*filter->x_perv[1]) + (filter->b2*filter->x_perv[2]) -
           (filter->a1*filter->y_perv[1]) - (filter->a2*filter->y_perv[2]);
//  printf("HPF-output:%d\n",output);
  //输出历史更新
  filter->y_perv[2] = filter->y_perv[1];
  filter->y_perv[1] = output;
  
  
  return output;
}

/*********************************************************************************************************
* 函数名称：MovingAverage_Init
* 函数功能：初始化移动平均滤波器
* 输入参数：*filter-滑动平均滤波器结构体 *buffer-数据缓冲区 window_size-窗大小
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
void MovingAverage_Init(MovingAverageFilter *filter, float *buffer, u16 window_size)
{
  int i = 0;
  
  filter->buffer = buffer;           //指定滑动平均滤波器数据缓冲区
  filter->window_size = window_size; //指定窗大小
  filter->index = 0;                 //当前写入位置为0
  filter->filled = 0;                //缓冲区未满
  filter->sum = 0;
  
  for (i = 0; i < window_size; i++)  //初始化数据缓冲区
  {
    buffer[i] = 0; // 直接对每个元素赋0
  }
}
/*********************************************************************************************************
* 函数名称：MovingAverageFilter_Pro
* 函数功能：平滑滤波
* 输入参数：*filter-结构体 input-待滤波样点
* 输出参数：output
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
float  MovingAverageFilter_Pro(MovingAverageFilter *filter, float input)
{
  u16 divisor;                                              //当前缓冲区数据个数
  float output = 0;
  if (input < -32768 || input > 32767) 
    {
//    printf("ERR: input out of range %0.2f\n", input);
//    while(1);
      input = 0;
    }
    if ((u32)filter->buffer < 0x20000000 || (u32)filter->buffer > 0x20020000) {
        printf("ERR: buffer=0x%p invalid!\n", filter->buffer);
        while(1);
    }
    if (filter->index >= filter->window_size) {
        printf("ERR: index=%d >= size=%d\n", filter->index, filter->window_size);
        while(1);
    }
  if(filter->filled)                                        //当缓冲区已满
  {
    filter->sum -= filter->buffer[filter->index];           //和减去上一个数据点
  }
  filter->sum += input;                                     //加上当前数据点  

  filter->buffer[filter->index] = input;                    //更新缓冲区数据  111
  filter->index = (filter->index + 1) % filter->window_size;//计算索引 实现0-window_size循环
  //当filled为0 index为0 时判断缓冲区已满
  if(!filter->filled && (filter->index == 0))               
  {
    filter->filled = 1;                                     //将filled置一
  }
  
  //当filled为1时 divisor=window_size;filled为0时 divisor=filter->index + 1
  divisor = filter->filled ? filter->window_size : (filter->index + 1);
  output = filter->sum / divisor;                          //计算均值
  
  return output;
}

/*********************************************************************************************************
* 函数名称：FilterInit
* 函数功能：初始化Filter模块 创建对应滤波器结构体
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
void FilterInit(void)
{
  
}

/*********************************************************************************************************
* 函数名称：
* 函数功能：
* 输入参数：
* 输出参数：
* 返 回 值：
* 创建日期：
* 注    意：
*********************************************************************************************************/
