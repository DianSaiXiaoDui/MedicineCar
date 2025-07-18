/*
 * DC.c
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#include "DC.h"
#include "math.h"
#include "Velocity_PID.h"

void DC_Init()
{
    //电机1 AIN2=0，电机2 AIN4=0
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    //初始化占空比置0
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	//启动编码器
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   // 启动左轮编码器
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);   // 启动右轮编码器
	//编码器计数值置0
	__HAL_TIM_SET_COUNTER(&htim1,0);
	__HAL_TIM_SET_COUNTER(&htim8,0);
}

//左后轮设置速度
void BL_SetVelocity(float BL_ratio)
{
   //ratio:最大电机转速占比（范围0~1）
   uint8_t BL_dir;//左后轮转动方向
    /*转速限制*/
   if(BL_ratio>DC_RATIO_MAX)
      BL_ratio=DC_RATIO_MAX;
   else if(BL_ratio<-DC_RATIO_MAX)
      BL_ratio=-DC_RATIO_MAX;

   /*方向判断*/
   if(BL_ratio>0)
       BL_dir=1;//正转
   else if(BL_ratio<0)
       BL_dir=0;//反转

    /*正转驱动*/
    if(BL_dir>0)
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(uint16_t)(BL_ratio*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   //AIN2=0
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }

    /*反转驱动*/
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(uint16_t)((1-(-BL_ratio))*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);   //AIN2= 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    }
}

//右后轮设置速度
void BR_SetVelocity(float BR_ratio)
{
   //ratio:最大电机转速占比（范围0~1）
   uint8_t BR_dir;//左后轮转动方向
    /*转速限制*/
   if(BR_ratio>DC_RATIO_MAX)
       BR_ratio=DC_RATIO_MAX;
   else if(BR_ratio<-DC_RATIO_MAX)
       BR_ratio=-DC_RATIO_MAX;

   /*方向判断*/
   if(BR_ratio>0)
       BR_dir=1;//正转
   else if(BR_ratio<0)
       BR_dir=0;//反转

    /*正转驱动*/
    if(BR_dir>0)
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,(uint16_t)(BR_ratio*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   //AIN4=0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }

    /*反转驱动*/
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,(uint16_t)((1-(-BR_ratio))*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   //AIN4= 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    }
}

//设置速度
void SetVelocity(float BL_ratio,float BR_ratio)
{
	//若电机未启动，启动电机
	if(MoveFlag==0)
	{
		//启动两路pwm输出
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
		//两路pwm占空比置0
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
		//编码器计数值置0
		__HAL_TIM_SET_COUNTER(&htim1,0);
		__HAL_TIM_SET_COUNTER(&htim8,0);
		MoveFlag=1;
	}
	if(MoveFlag==1)
	{
		//速度小到一定程度时，停止
		/*if(fabs(BL_ratio)<DC_RATIO_MIN && fabs(BR_ratio)<DC_RATIO_MIN && DCStopFlag==1)
		{
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		    DCStopFlag=0;
		    Velocity_PID_Reset();
		    MoveFlag=0;
		}
		else
		{
			BL_SetVelocity(BL_ratio);
			BR_SetVelocity(BR_ratio);
		}
		*/
		BL_SetVelocity(BL_ratio);
	    BR_SetVelocity(BR_ratio);
	}
}


//获取速度,计算一个测速周期内的车后轮速度和整体速度，更新移动距离
void GetVelocity(void)
{
   double circle=PI*DD/10;//轮子周长（cm）
   double dpp=circle/56000;  //每个脉冲对应的距离（cm）distance per pulse


   int16_t delta_distance_BL,delta_distance_BR;//一个测速周期内计数器累加值
   float delta_distance_BL_cm,delta_distance_BR_cm;//将计数器累加值转化为cm

    //获取一个测速周期内左后轮/右后轮计数器值
   delta_distance_BL =  __HAL_TIM_GET_COUNTER(&htim1);
   delta_distance_BR =  __HAL_TIM_GET_COUNTER(&htim8);

    //将计时器清零，开始新一轮计数
   __HAL_TIM_SET_COUNTER(&htim1,0);
   __HAL_TIM_SET_COUNTER(&htim8,0);

   //将计数器值转化为移动距离（cm）
   delta_distance_BL_cm=delta_distance_BL*dpp;
   delta_distance_BR_cm=delta_distance_BR*dpp;

   //计算一个测速周期内的速度
   v_BL=delta_distance_BL_cm/(T_velocity/1000.0);
   v_BR=delta_distance_BR_cm/(T_velocity/1000.0);

   //取小车两后轮速度平均值计算小车整体速度
   v_C=(v_BL+v_BR)/2;

	/*if(fabs(v_BL)<10 && fabs(v_BR)<10 && DCStopFlag==1)
	{
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	    DCStopFlag=0;
	    Velocity_PID_Reset();
	    MoveFlag=0;
	}*/

   //计算一个测速周期内小车移动距离,更新总距离
    if(MoveFlag==1 && DistanceFlag==1)
    {
    	TotalDistance+=(v_C*(T_velocity/1000.0));
    	if(TotalDistance>=TargetDistance)
    	{
    		StraightStopFlag=1;
    	}
    }
}

void DC_Turn(int8_t dir,uint16_t angle)
{
	//float w=angle/time*PI/180.0;
	//float v=K/2*w;//旋转线速度
	//float ratio=(0.54+v)/107.75;
	//顺时针转90度
	if(dir>0 && angle==90)
	{
		SetVelocity(0.1,-0.1);
		Set_TargetVelocity(20,-20);
		TurnFlag=1;
		TurnPeriod=650;
		//DC_Stop();
	}
	//逆时针转90度
	else if(dir<0 && angle==90)
	{
		SetVelocity(-0.1,0.1);
		Set_TargetVelocity(-20,20);
		TurnFlag=1;
		TurnPeriod=650;
		//DC_Stop();
	}
	//掉头
	if(angle==180)
	{
		SetVelocity(0.1,-0.1);
		Set_TargetVelocity(20,-20);
		TurnFlag=1;
		TurnPeriod=1100;
		//DC_Stop();
	}
}

void DC_Forward(float Distance,uint8_t inf)//前进
{
	Set_TargetVelocity(20,20);//设置pid目标速度20
	SetVelocity(0.1,0.1);//启动电机
	if(inf==0)//走一定距离后停下
	{
		  DistanceFlag=1;
		  TotalDistance=0;
		  TargetDistance=Distance;
	}
}
void DC_Backward(float Distance,uint8_t inf)//后退
{
	Set_TargetVelocity(-20,-20);//设置pid目标速度20
    SetVelocity(-0.1,-0.1);//启动电机
	if(inf==0)//走一定距离后停下
	{
		  DistanceFlag=1;
		  TotalDistance=0;
		  TargetDistance=Distance;
	}
}

void DC_Stop(void)//电机停止转动
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
	//HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	DCStopFlag=0;
	Velocity_PID_Reset();
	MoveFlag=0;
}



