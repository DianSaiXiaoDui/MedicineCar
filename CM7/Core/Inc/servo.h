/*
 * servo.h
 *
 *  Created on: Jul 9, 2025
 *      Author: 21316
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include "math.h"
#include "tim.h"
//#define PI 3.141592653589793238462643383279

/*参数为角度的正切函数*/
float tanA(float angle);
/*角度转CCR*/
uint16_t Angle2CCR(float angle);
/*设置舵机角度函数（180度舵机）*/
void SetAngle(float angle);

#endif /* INC_SERVO_H_ */
