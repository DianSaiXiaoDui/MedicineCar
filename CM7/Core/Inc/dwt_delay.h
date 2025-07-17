#ifndef __DWT_DELAY_H
#define __DWT_DELAY_H

#include "main.h"

uint8_t DWT_Init(void);//DWT初始化配置
void DWT_Delay_us(uint32_t us);//基于DWT的延时1us函数
uint32_t measure_actual_us(uint32_t us);

#endif
