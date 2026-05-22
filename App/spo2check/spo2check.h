#ifndef __SPO2CHECK_H_
#define __SPO2CHECK_H_

#include "UART1.h"


//void SendAdjCmd(u16 spo_ir,u16 spo_red);
void ResetAdj(void);//复位调光参数，准备重新调光
void RoughAdj(void);//粗调红外光强和红光光强
void JudgeReAdj(void);//判断是否需要重新调光
void FineAdj(void);//细调红外光和红光光强
void FINGERtest(int irdata,int reddata,int iravg,int redavg);
void Init_SPO2(void);
u16 GetIRDA(void);
u16 GetRedDA(void);
void min(float filtered_data,int tag);//求最小峰值
void max(float filtered_data,int tag);//求最大峰值
short calc_spo2(void);
int GetIRavg(void);
int GetREDavg(void);
void RESETdata(void);
//float  MovingAverageFilter_Pro(MovingAverageFilter *filter, float input);
//float FilterPro(Filter *filter, float input);

#endif
