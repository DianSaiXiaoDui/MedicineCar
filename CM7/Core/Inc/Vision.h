/*
 * Vision.h
 *
 *  Created on: Jul 10, 2025
 *      Author: 21316
 */

#ifndef INC_VISION_H_
#define INC_VISION_H_

#include "main.h"
#include "usart.h"

void RecoginzeNumSingle(uint8_t* num);//药房，识别一个数字

void RecoginizeNumDouble(uint8_t* num_Left,uint8_t* num_right); //交叉处，识别左右两个数字

#endif /* INC_VISION_H_ */
