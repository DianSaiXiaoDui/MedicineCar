/*
 * Location_PID.h
 *
 *  Created on: Jul 10, 2025
 *      Author: 21316
 */

#ifndef INC_LOCATION_PID_H_
#define INC_LOCATION_PID_H_

#include "main.h"

//位置环PID结构体（位置环+速度环串级双闭环）
typedef struct {
    // 速度环参数
    float Kp;               // 比例系数（位置环）
    float Ki;               // 积分系数（位置环）
    float Kd;               // 微分系数（位置环）
    float CurLocation;         // 当前位置
    float TargetLocation;      // 目标位置
    float Output;           // 输出值(作为速度环目标速度)
    float Error0;           // 当前位置误差
    float Error1;           // 上一位置误差
    float ErrorInt;         // 累计位置误差
    float ErrorIntThresh;   // 积分限幅
    uint32_t OutputHighThresh; // 输出限幅上界
    uint32_t OutputLowThresh;  // 输出限幅下界
    uint8_t Reset;
    float LD;
    float I;


} Location_PID_Struct;

extern Location_PID_Struct Location_PID;

//位置环pid初始化函数
void Location_PID_Init();

void Location_PID_Reset();

#endif /* INC_LOCATION_PID_H_ */
