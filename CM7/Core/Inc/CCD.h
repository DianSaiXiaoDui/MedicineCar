#ifndef __CCD_H__
#define __CCD_H__

#include "main.h"
#include "dwt_delay.h"
#include "adc.h"

#define CCD_SI_H HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
#define CCD_SI_L HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
#define CCD_CLK_H HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
#define CCD_CLK_L HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);

void Linear_CCD_Read(uint16_t* CCDADV);//一次性读取线阵CCD128像素值
uint16_t ADC_GetValue();//ADC单通道单次转换读取值

void Dly_us(void);
void Dly(void);
void CCD_Read(uint16_t* CCDADV) ;
void Linear_CCD_Flush(void);


#endif
