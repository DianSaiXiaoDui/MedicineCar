/*
 * Velocity_PID.h
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#ifndef INC_VELOCITY_PID_H_
#define INC_VELOCITY_PID_H_

#include "main.h"
#include "tim.h"

//左轮速度pid结构体
typedef struct{
	    float Kp;               // 左轮比例系数
	    float Ki;               // 左轮积分系数
	    float Kd;               // 左轮微分系数
	    float P;                // 左轮比例项
	    float I;                // 左轮积分项
	    float D;                // 左轮微分项
	    float IThresh;          // 左轮积分限幅
	    float Error0;           // 左轮当前速度误差
	    float Error1;           // 左轮上一速度误差
	    float ErrorThresh;      // 左轮抗积分饱和临界误差
	    float ErrorInt;         // 左轮累计速度误差
	    float CurVelocity;      // 左轮当前速度
	    float TargetVelocity;   // 左轮目标速度
	    float PwmDuty;          // 左轮pid输出量:左电机pwm占空比
	    float OutputThreshH;    // 左轮pid输出限幅（上界）
	    float OutputThreshL;    // 左轮pid输出限幅（下界）
	    uint8_t Reset;        //左轮切换目标速度标志
}  BL_Velocity_PID_Struct;

//右轮速度pid结构体
typedef struct{
	    float Kp;               // 右轮比例系数
	    float Ki;               // 右轮积分系数
	    float Kd;               // 右轮微分系数
	    float P;                // 右轮比例项
	    float I;                // 右轮积分项
	    float D;                // 右轮微分项
	    float IThresh;          // 右轮积分限幅
	    float Error0;           // 右轮当前速度误差
	    float Error1;           // 右轮上一速度误差
	    float ErrorInt;         // 右轮累计速度误差
	    float ErrorThresh;      // 右轮抗积分饱和临界误差
	    float CurVelocity;      // 右轮当前速度
	    float TargetVelocity;   // 右轮目标速度
	    float PwmDuty;          // 右轮pid输出量:右电机pwm占空比
	    float OutputThreshH;    // 右轮pid输出大小限幅（上界）
	    float OutputThreshL;    // 右轮pid输出大小限幅（下界）
	    uint8_t Reset;        //右轮切换目标速度标志
}  BR_Velocity_PID_Struct;


extern BL_Velocity_PID_Struct BL_Velocity_PID;
extern BR_Velocity_PID_Struct BR_Velocity_PID;

//速度pid初始化
void BL_Velocity_PID_Init(void);
void BR_Velocity_PID_Init(void);
void Velocity_PID_Init(void);

//设置速度pid目标速度
void Set_BL_TargetVelocity(float v);
void Set_BR_TargetVelocity(float v);
void Set_TargetVelocity(float vl,float vr);

//设置速度pid参数
void Set_BL_Kp(float kp);
void Set_BL_Ki(float ki);
void Set_BL_Kd(float kd);
void Set_BR_Kp(float kp);
void Set_BR_Ki(float ki);
void Set_BR_Kd(float kd);

//速度pid控制
void BL_Velocity_PID_Control(void);
void BR_Velocity_PID_Control(void);
void Velocity_PID_Control(void);

//速度pid更新左右轮转速
void Velocity_PID_Update(void);
//void Velocity_Update_A(void);

//速度pid复位
void Velocity_PID_Reset(void);
void BL_Velocity_PID_Reset(void);
void BR_Velocity_PID_Reset(void);


//获取当前目标速度
float Get_BL_TargetVelocity();
float Get_BR_TargetVelocity();




#endif /* INC_VELOCITY_PID_H_ */
