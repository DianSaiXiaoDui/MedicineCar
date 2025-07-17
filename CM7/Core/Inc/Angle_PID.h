/*
 * Angle_PID.h
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#ifndef INC_ANGLE_PID_H_
#define INC_ANGLE_PID_H_

#include "CCD.h"

//转向pid结构体定义
typedef struct {
  float Kp;//比例系数
  float Ki;
  float Kd;//微分系数
  int16_t Error0;//当前误差
  int16_t Error1;//上一误差
  float ErrorInt;//累计误差
  float I;//积分项
  float AD;//微分项
  float IThresh;//积分限幅
  float OutputThresh;//输出限幅
  float Output;//输出
  uint8_t Reset;
}Angle_PID_Struct;

extern Angle_PID_Struct Angle_PID;//转向pid结构体

void Angle_PID_Init(void);//初始化函数

void Angle_PID_Control(void);//转向环pid控制

void Angle_PID_Reset(void);
#endif /* INC_ANGLE_PID_H_ */
