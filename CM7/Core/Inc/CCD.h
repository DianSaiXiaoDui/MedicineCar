#ifndef __CCD_H__
#define __CCD_H__

#include "main.h"
#include "adc.h"

#define CCD_SI_H HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
#define CCD_SI_L HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
#define CCD_CLK_H HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
#define CCD_CLK_L HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);


/*void Dly_us(void);
void Dly(void);
void CCD_Read(void) ;
void Linear_CCD_Flush(void);
void CCD_DataProcess(void);
*/

#endif
