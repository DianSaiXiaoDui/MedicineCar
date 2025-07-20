/*
 * Angle_PID.c
 *
 *  Created on: Jul 9, 2025
 */
#include "Angle_PID.h"
#include "Velocity_PID.h"

Angle_PID_Struct Angle_PID;//转向pid结构体
extern uint16_t regularVelocity;
void Angle_PID_Init(void)
{
   Angle_PID.Kp=0.03;
   Angle_PID.Ki=0.0;
   Angle_PID.Kd=0.0;
   Angle_PID.P = 0;
   Angle_PID.I = 0;
   Angle_PID.D = 0;
   Angle_PID.Error0=0;
   Angle_PID.Error1=0;
   Angle_PID.ErrorThresh = 0;
   Angle_PID.ErrorInt=0;
   Angle_PID.IThresh=0;
   Angle_PID.CurX = 0;
   Angle_PID.TargetX = 320;
   Angle_PID.deltaVelocity = 0;
   Angle_PID.OutputThreshH=5;
   Angle_PID.OutputThreshL = 0;
   Angle_PID.Reset=0;
}

/* Private PID functions ---------------------------------------------------------*/
//转向pid调控：输出和速度pid输出同数量级
void Angle_PID_Control()
{
	Angle_PID.Error0 = Angle_PID.TargetX - Angle_PID.CurX;//更新当前横向坐标误差

	//计算比例项
	Angle_PID.P=Angle_PID.Kp*Angle_PID.Error0;

	//计算积分项
	if(Angle_PID.Reset==1)//切换目标速度后，累计误差置0
	{
		Angle_PID.ErrorInt=0;
	}

	//当误差小于临界误差时，更新误差积分项
	if(MoveFlag == 1) {
		if(Angle_PID.Error0<Angle_PID.ErrorThresh)
			Angle_PID.ErrorInt += Angle_PID.Error0; // 小车运动时累加误差
	}

	Angle_PID.I=Angle_PID.Ki*Angle_PID.ErrorInt;

	// 积分限幅
	clip(&Angle_PID.I,0,Angle_PID.IThresh);

	//计算微分项
	if(Angle_PID.Reset==1)//切换目标速度后，第一次pid计算微分项为0，防止过冲
	{
	   Angle_PID.D=0;
	   Angle_PID.Reset=0;
	}
	else
	{
	   Angle_PID.D=Angle_PID.Kd*(Angle_PID.Error0-Angle_PID.Error1);
	}

	// 计算输出值 (PI控制)
	Angle_PID.deltaVelocity = Angle_PID.P + Angle_PID.I + Angle_PID.D;

	// 输出限幅
	clip(&Angle_PID.deltaVelocity,Angle_PID.OutputThreshL,Angle_PID.OutputThreshH);

	// 更新上一误差
	Angle_PID.Error1 = Angle_PID.Error0;
}

void Angle_PID_Reset()
{
	Angle_PID.Reset=1;
}

void Angle_PID_SetCurX(uint16_t CurX)
{
	Angle_PID.CurX=CurX;
}

void Angle_PID_Update()
{
	Angle_PID_Control();
    float new_BL_Vel = regularVelocity - Angle_PID.deltaVelocity;
	float new_RL_Vel = regularVelocity + Angle_PID.deltaVelocity;
	Set_TargetVelocity(new_BL_Vel, new_RL_Vel);
}
