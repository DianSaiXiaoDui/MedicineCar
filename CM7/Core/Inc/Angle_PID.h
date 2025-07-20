/*
 * Angle_PID.h
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#ifndef INC_ANGLE_PID_H_
#define INC_ANGLE_PID_H_

#include "main.h"

//转向pid结构体定义
typedef struct{
	    float Kp;               // 比例系数
	    float Ki;               // 积分系数
	    float Kd;               // 微分系数
	    float P;                // 比例项
	    float I;                // 积分项
	    float D;                // 微分项
	    float IThresh;          // 积分限幅
	    float Error0;           // 当前横向误差
	    float Error1;           // 上一横向误差
	    float ErrorThresh;      // 抗积分饱和临界误差
	    float ErrorInt;         // 累计横向误差
	    uint16_t CurX;      // 当前横向坐标
	    uint16_t TargetX;   // 目标横向坐标
	    float deltaVelocity;    // pid输出量:两轮的差速
	    float OutputThreshH;    // pid输出限幅（上界）
	    float OutputThreshL;    // pid输出限幅（下界）
	    uint8_t Reset;        	//切换目标标志
}Angle_PID_Struct;

extern Angle_PID_Struct Angle_PID;//转向pid结构体

void Angle_PID_Init(void);//初始化函数

void Angle_PID_Control(void);//转向环pid控制

void Angle_PID_Reset(void);

void Angle_PID_Update(void);

void Angle_PID_SetCurX(uint16_t CurX);

#endif /* INC_ANGLE_PID_H_ */
