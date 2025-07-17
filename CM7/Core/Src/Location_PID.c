/*
 * Location_PID.c
 *
 *  Created on: Jul 10, 2025
 *      Author: 21316
 */

#include "Location_PID.h"
#include "Velocity_PID.h"

Location_PID_Struct Location_PID;

void Location_PID_Init()
{
   Location_PID.Kp=0.12;
   Location_PID.Ki=0.0;
   Location_PID.Kd=0.0;
   Location_PID.CurLocation=0;
   Location_PID.TargetLocation=0;
   Location_PID.Output=0;
   Location_PID.OutputHighThresh=50;
   Location_PID.OutputLowThresh=5;
   Location_PID.LD=0;
   Location_PID.Reset=0;
   Location_PID.I=0;
}

//位置环pid
void Location_PID_Control(void)
{
    //current_position = Get_Encoder_Position();
    //float Distance_T=current_position/1000*0.75;
    Location_PID.Error0=Location_PID.TargetLocation-Location_PID.CurLocation;//计算当前误差
    Location_PID.ErrorInt+=Location_PID.Error0;//小车运动时累加误差

    if(Location_PID.Error0<1)
    {
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		Velocity_PID_Reset();
		Location_PID_Reset();
		Angle_PID_Reset();
		MoveFlag=0;//停止运动
    }
    Location_PID.LD=Location_PID.Kd*(Location_PID.Error0-Location_PID.Error1);
    if(Location_PID.Reset==1)
    {
    	Location_PID.Reset=0;
    	Location_PID.LD=0;
    }
    //计算输出值(小车速度)
    Location_PID.Output=Location_PID.Kp*Location_PID.Error0+Location_PID.LD;

    //更新上一误差
    Location_PID.Error1=Location_PID.Error0;
    //输出限幅
    if(Location_PID.Output>Location_PID.OutputHighThresh)
      Location_PID.Output=Location_PID.OutputHighThresh;
    if(Location_PID.Output<Location_PID.OutputLowThresh)
      Location_PID.Output=Location_PID.OutputLowThresh;

}

void Location_PID_Reset()
{
  	Location_PID.Reset=1;
  	Location_PID.Error1=0;
  	Location_PID.ErrorInt=0;
}
