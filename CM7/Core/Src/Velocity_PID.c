/*
 * Velocity_PID.c
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#include "Velocity_PID.h"
#include "Angle_PID.h"
#include "DC.h"
#include "main.h"

BL_Velocity_PID_Struct BL_Velocity_PID;//左轮速度pid控制器
BR_Velocity_PID_Struct BR_Velocity_PID;//右轮速度pid控制器

//左轮速度pid初始化函数
void BL_Velocity_PID_Init(void)
{
    BL_Velocity_PID.Kp=0.5;               // 左轮比例系数
    BL_Velocity_PID.Ki=0.1;               // 左轮积分系数
    BL_Velocity_PID.Kd=0.05;               // 左轮微分系数
    BL_Velocity_PID.P=0.0;                // 左轮比例项
    BL_Velocity_PID.I=0.0;                // 左轮积分项
    BL_Velocity_PID.D=0.0;                // 左轮微分项
    BL_Velocity_PID.IThresh=50.0;          // 左轮积分限幅
    BL_Velocity_PID.Error0=0.0;           // 左轮当前速度误差
    BL_Velocity_PID.Error1=0.0;           // 左轮上一速度误差
    BL_Velocity_PID.ErrorThresh=50.0;      // 左轮抗积分饱和临界误差
    BL_Velocity_PID.ErrorInt=0.0;         // 左轮累计速度误差
    BL_Velocity_PID.CurVelocity=0.0;      // 左轮当前速度
    BL_Velocity_PID.TargetVelocity=20;   // 左轮目标速度
    BL_Velocity_PID.PwmDuty=0.0;          // 左轮pid输出量:左电机pwm占空比
    BL_Velocity_PID.OutputThreshH=50.0;    // 左轮pid输出限幅（上界）
    BL_Velocity_PID.OutputThreshL=0.0;    // 左轮pid输出限幅（下界）
    BL_Velocity_PID.Reset=0;             //左轮切换目标速度标志
}

void BR_Velocity_PID_Init(void)
{
	BR_Velocity_PID.Kp=0.5;               // 右轮比例系数
	BR_Velocity_PID.Ki=0.1;               // 右轮积分系数
	BR_Velocity_PID.Kd=0.05;               // 右轮微分系数
	BR_Velocity_PID.P=0.0;                // 右轮比例项
	BR_Velocity_PID.I=0.0;                // 右轮积分项
	BR_Velocity_PID.D=0.0;                // 右轮微分项
	BR_Velocity_PID.IThresh=50;          // 右轮积分限幅
	BR_Velocity_PID.Error0=0.0;           // 右轮当前速度误差
	BR_Velocity_PID.Error1=0.0;           // 右轮上一速度误差
	BR_Velocity_PID.ErrorInt=0.0;         // 右轮累计速度误差
	BR_Velocity_PID.ErrorThresh=50.0;      // 右轮抗积分饱和临界误差
	BR_Velocity_PID.CurVelocity=0.0;      // 右轮当前速度
	BR_Velocity_PID.TargetVelocity=20;   // 右轮目标速度
	BR_Velocity_PID.PwmDuty=0.0;          // 右轮pid输出量:右电机pwm占空比
	BR_Velocity_PID.OutputThreshH=50.0;    // 右轮pid输出大小限幅（上界）
	BR_Velocity_PID.OutputThreshL=0.0;    // 右轮pid输出大小限幅（下界）
	BR_Velocity_PID.Reset=0;             //右轮切换目标速度标志
}

void Velocity_PID_Init(void)
{
	BL_Velocity_PID_Init();
	BR_Velocity_PID_Init();
}


//左轮速度pid控制函数
void BL_Velocity_PID_Control(void)
{
	BL_Velocity_PID.CurVelocity=v_BL;//更新当前速度
    BL_Velocity_PID.Error0 = BL_Velocity_PID.TargetVelocity - BL_Velocity_PID.CurVelocity;//更新当前速度误差

    //计算比例项
    BL_Velocity_PID.P=BL_Velocity_PID.Kp*BL_Velocity_PID.Error0;

    //计算积分项
    if(BL_Velocity_PID.Reset==1)//切换目标速度后，累计误差置0
    {
    	BL_Velocity_PID.ErrorInt=0;
    }

    //当误差小于临界误差时，更新误差积分项
    if(MoveFlag == 1) {
    	if(fabs(BL_Velocity_PID.Error0)<BL_Velocity_PID.ErrorThresh)
            BL_Velocity_PID.ErrorInt += BL_Velocity_PID.Error0; // 小车运动时累加误差
    }

    BL_Velocity_PID.I=BL_Velocity_PID.Ki*BL_Velocity_PID.ErrorInt;

    // 积分限幅
    clip(&BL_Velocity_PID.I,0,BL_Velocity_PID.IThresh);

    //计算微分项
    if(BL_Velocity_PID.Reset==1)//切换目标速度后，第一次pid计算微分项为0，防止过冲
    {
       BL_Velocity_PID.D=0;
       BL_Velocity_PID.Reset=0;
    }
    else
    {
       BL_Velocity_PID.D=BL_Velocity_PID.Kd*(BL_Velocity_PID.Error0-BL_Velocity_PID.Error1);
    }

    // 计算输出值 (PI控制)
    BL_Velocity_PID.PwmDuty = BL_Velocity_PID.P + BL_Velocity_PID.I + BL_Velocity_PID.D;

    // 输出限幅
    clip(&BL_Velocity_PID.PwmDuty,BL_Velocity_PID.OutputThreshL,BL_Velocity_PID.OutputThreshH);

    // 更新上一误差
    BL_Velocity_PID.Error1 = BL_Velocity_PID.Error0;
}

//右轮速度pid控制函数
void BR_Velocity_PID_Control(void)
{
	BR_Velocity_PID.CurVelocity=v_BR;//更新当前速度
    BR_Velocity_PID.Error0 = BR_Velocity_PID.TargetVelocity - BR_Velocity_PID.CurVelocity;//更新当前速度误差

    //计算比例项
    BR_Velocity_PID.P=BR_Velocity_PID.Kp*BR_Velocity_PID.Error0;

    //计算积分项
    if(BR_Velocity_PID.Reset==1)//切换目标速度后，累计误差置0
    {
    	BR_Velocity_PID.ErrorInt=0;
    }

    //当误差小于临界误差时，更新误差积分项
    if(MoveFlag == 1) {
    	if(BR_Velocity_PID.Error0<BR_Velocity_PID.ErrorThresh)
            BR_Velocity_PID.ErrorInt += BR_Velocity_PID.Error0; // 小车运动时累加误差
    }

    BR_Velocity_PID.I=BR_Velocity_PID.Ki*BR_Velocity_PID.ErrorInt;

    // 积分限幅
    clip(&BR_Velocity_PID.I,0,BR_Velocity_PID.IThresh);

    //计算微分项
    if(BR_Velocity_PID.Reset==1)//切换目标速度后，第一次pid计算微分项为0，防止过冲
    {
       BR_Velocity_PID.D=0;
       BR_Velocity_PID.Reset=0;
    }
    else
    {
       BR_Velocity_PID.D=BR_Velocity_PID.Kd*(BR_Velocity_PID.Error0-BR_Velocity_PID.Error1);
    }

    // 计算输出值 (PI控制)
    BR_Velocity_PID.PwmDuty = BR_Velocity_PID.P + BR_Velocity_PID.I + BR_Velocity_PID.D;

    // 输出限幅
    clip(&BR_Velocity_PID.PwmDuty,BR_Velocity_PID.OutputThreshL,BR_Velocity_PID.OutputThreshH);

    // 更新上一误差
    BR_Velocity_PID.Error1 = BR_Velocity_PID.Error0;
}


//调用pid控制函数，根据pid输出量更新左右轮速度
void Velocity_Update(void)
{
	BL_Velocity_PID_Control();
	BR_Velocity_PID_Control();
    SetVelocity(BL_Velocity_PID.PwmDuty/100.0,BR_Velocity_PID.PwmDuty/100.0);//将占空比转化为小数形式，调用电机设速函数
}

//结合速度pid基准速度+转向pid差速更新左右后轮速度
/*
void Velocity_Update_A(void)
{
	 float DC_CCR_BR=Velocity_PID.Output-Angle_PID.Output;
	 float DC_CCR_BL=Velocity_PID.Output+Angle_PID.Output;

    // 归一化到比例值 (-1.0 到 1.0)
	 float velocity_ratio_BR = DC_CCR_BR *100 / DC_ARR;//计算右后轮转速比例
	 float velocity_ratio_BL = DC_CCR_BL *100 / DC_ARR;//计算左后轮转速比例

	 BL_SetVelocity(velocity_ratio_BL);//设置左后轮转速
	 BR_SetVelocity(velocity_ratio_BR);//设置右后轮转速
}
*/

void BL_Velocity_PID_Reset(void)
{
	BL_Velocity_PID.Reset=1;
}

void BR_Velocity_PID_Reset(void)
{
	BR_Velocity_PID.Reset=1;
}

void Velocity_PID_Reset(void)
{
	BR_Velocity_PID.Reset=1;
	BL_Velocity_PID.Reset=1;
}


//设置速度pid目标速度
void Set_BL_TargetVelocity(float v)
{
	BL_Velocity_PID.TargetVelocity=v;
}

void Set_BR_TargetVelocity(float v)
{
    BR_Velocity_PID.TargetVelocity=v;
}

void Set_TargetVelocity(float vl,float vr)
{
	BL_Velocity_PID.TargetVelocity=vl;
	BR_Velocity_PID.TargetVelocity=vr;
}

float Get_BL_TargetVelocity()
{
	return BL_Velocity_PID.TargetVelocity;
}

float Get_BR_TargetVelocity()
{
	return BR_Velocity_PID.TargetVelocity;
}

void Set_BL_Kp(float kp)
{
	BL_Velocity_PID.Kp=kp;
}
void Set_BL_Ki(float ki)
{
	BL_Velocity_PID.Ki=ki;
}
void Set_BL_Kd(float kd)
{
	BL_Velocity_PID.Kd=kd;
}

void Set_BR_Kp(float kp)
{
	BR_Velocity_PID.Kp=kp;
}

void Set_BR_Ki(float ki)
{
	BR_Velocity_PID.Ki=ki;
}
void Set_BR_Kd(float kd)
{
	BR_Velocity_PID.Kd=kd;
}
