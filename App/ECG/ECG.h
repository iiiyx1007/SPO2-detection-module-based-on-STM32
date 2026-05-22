#ifndef __ECG_H_
#define __ECG_H_

void Init_ECG(void);  //初始化导联脱落检测引脚
float notch_filter(float x); //50Hz陷波
float IIRFilterECG(float x); //IIR滤波
float FIRFilterECG(float x); //FIR滤波 
void detect_peak(float filtered_data);  // 峰值检测
//void min(float filtered_data,int tag);//求最小峰值
//void max(float filtered_data,int tag);//求最大峰值
short calc_pulse_rate(void);//计算脉率
//short calc_spo2(void);
//int GetIRavg(void);
//int GetREDavg(void);
//short calc_heart_rate(void);  //计算心率

#endif
