/*
 * servo.c
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */
#include "servo.h"


/*参数为角度的正切函数*/
float tanA(float angle)
{
    float radian=angle/180*PI;//角度转弧度
    return tan(radian);//返回正切值
}

/*根据转角计算舵机pwm,假设180度舵机，角度范围0~180°,对应CCR_min~CCR_max*/
uint16_t Angle2CCR(float angle)
{
    float CCR=(Servo_ARR/1800.0)*angle+Servo_ARR/40.0;
    return (uint16_t)CCR;
}


/*设置舵机角度函数*/
void SetAngle(float angle)
{
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Angle2CCR(angle));
}

