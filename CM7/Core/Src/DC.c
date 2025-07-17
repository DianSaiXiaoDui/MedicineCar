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
    }

    /*反转驱动*/
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(uint16_t)((1-(-BL_ratio))*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);   //AIN2= 1
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
    }

    /*反转驱动*/
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,(uint16_t)((1-(-BR_ratio))*DC_ARR));//设置PWM占空比BL_ratio
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   //AIN4= 1
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
		if(fabs(BL_ratio)<DC_RATIO_MIN && fabs(BR_ratio)<DC_RATIO_MIN)
		{
			HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}
		else
		{
			BL_SetVelocity(BL_ratio);
			BR_SetVelocity(BR_ratio);
		}
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

   if(fabs(v_BL)<0.000001 && fabs(v_BR)<0.000001)
   {
	   MoveFlag=0;
	   Velocity_PID_Reset();
   }
   //取小车两后轮速度平均值计算小车整体速度
   v_C=(v_BL+v_BR)/2;

   //计算一个测速周期内小车移动距离,更新总距离
    if(MoveFlag==1)
    {
    	Totaldistance+=(v_C*(T_velocity/1000.0));
    }
}

void Turn(uint8_t dir,float time)
{
	/*float w=angle/time*PI/180.0;//旋转角速度
	float v=K/2*w;//旋转线速度
	float ratio=(0.54+v)/107.75;*/
	//顺时针
	if(dir>0)
	{
		SetVelocity(0.1,-0.1);
		HAL_Delay((uint32_t)time*1000);
	}
	//逆时针
	else if(dir<0)
	{
		SetVelocity(-0.1,0.1);
		HAL_Delay((uint32_t)time*1000);
	}
}

void DC_Stop(void)//电机停止转动
{
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,0);
	MoveFlag=0;
	Velocity_PID_Reset();
}


