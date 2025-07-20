/*
 * DC.h
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#ifndef INC_DC_H_
#define INC_DC_H_

#include "main.h"
#include "tim.h"
#include "math.h"

void DC_Init();


//左后轮设置速度
void BL_SetVelocity(float BL_ratio);

//右后轮设置速度
void BR_SetVelocity(float BR_ratio);

//设置速度
void SetVelocity(float BL_ratio,float BR_ratio);

//获取速度,计算一个测速周期内的车后轮速度和整体速度，更新移动距离
void GetVelocity(void);

//以一定速度原地转弯一定时间（堵塞式）
//dir=0 向左 dir=1 向右,angle：角度，time:希望的时间；
void Turn(int8_t dir,uint16_t angle);

void DC_Forward(float Distance,uint8_t inf);//前进
void DC_Backward(float Distance,uint8_t inf);//后退
void DC_Stop(void);//电机停止转动
void DC_Start(uint8_t dir);//电机启动


#endif /* INC_DC_H_ */
