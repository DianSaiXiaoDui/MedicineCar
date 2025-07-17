/*
 * Angle_PID.c
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */
#include "Angle_PID.h"


Angle_PID_Struct Angle_PID;//转向pid结构体

void Angle_PID_Init(void)
{
   //ADC校正
   HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED );

	//PID初始化
  //0.5、0.16->0.48、0.14
   Angle_PID.Kp=0.3;
   Angle_PID.Ki=0.0;
   Angle_PID.Kd=0.0;
   Angle_PID.I=0;
   Angle_PID.AD=0;
   Angle_PID.Error0=0;
   Angle_PID.Error1=0;
   Angle_PID.ErrorInt=0;
   Angle_PID.IThresh=5;
   Angle_PID.OutputThresh=10;
   Angle_PID.Output=0;
   Angle_PID.Reset=0;
}

/* Private PID functions ---------------------------------------------------------*/
//转向pid调控：输出和速度pid输出同数量级
void Angle_PID_Control()
{

    Angle_PID.Error0 = TargetIdx -  CenterIdx;

    if(MoveFlag==1 ) //开始运动时积分项起作用
    {
         Angle_PID.ErrorInt+=Angle_PID.Error0;
         //积分限幅
         Angle_PID.I=Angle_PID.Ki*Angle_PID.ErrorInt;
         if(Angle_PID.I>Angle_PID.IThresh)
             Angle_PID.I=Angle_PID.IThresh;
         if(Angle_PID.I<-Angle_PID.IThresh)
              Angle_PID.I=-Angle_PID.IThresh;
    }
    else{
       Angle_PID.I=0;
    }

    Angle_PID.AD=Angle_PID.Kd*(Angle_PID.Error0-Angle_PID.Error1);

    if(Angle_PID.Reset==1)
    {
    	Angle_PID.Reset=0;
    	Angle_PID.AD=0;
    }

   Angle_PID.Output=  Angle_PID.Kp*Angle_PID.Error0 +Angle_PID.I+Angle_PID.AD;  //舵机pid输出值，2倍CCR
   Angle_PID.Error1=Angle_PID.Error0;

   //输出限幅
   if(Angle_PID.Output>Angle_PID.OutputThresh)
	   Angle_PID.Output=Angle_PID.OutputThresh;
   else if(Angle_PID.Output<-Angle_PID.OutputThresh)
	   Angle_PID.Output=-Angle_PID.OutputThresh;

}

void Angle_PID_Reset()
{
	Angle_PID.Reset=1;
	Angle_PID.ErrorInt=0;
	Angle_PID.Error1=0;
}
