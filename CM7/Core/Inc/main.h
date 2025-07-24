/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "CCD.h"
#include "servo.h"
#include "DC.h"
#include "Velocity_PID.h"
#include "Angle_PID.h"
#include "Location_PID.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PI 3.141592653589793238
#define K 11.9 //轴距11.9cm
#define L 20 //车长20cm
#define DD 65 //车直�?????????65mm
#define K1 0.25 //系数K/2L
#define Servo_ARR 10000//舵机pwm定时器ARR
#define DC_ARR 10000//电机pwm定时器ARR
#define DC_RATIO_MAX 0.9//�??????????????大电机转速（比例，范�??????????????0~1�??????????????
#define DC_RATIO_MIN 0.01//�??????????????大电机转速（比例，范�??????????????0~1�??????????????
#define V_BASE 60 //运动速度 cm/s
#define V_MIN 5   //串口屏可调最小运动�?�度
#define V_MAX 100 //串口屏可调最大目标�?�度
#define CenterIdx 63 //机械中心

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CN_Pin GPIO_PIN_6
#define SPI2_CN_GPIO_Port GPIOF
#define SPI2_IRQ_Pin GPIO_PIN_8
#define SPI2_IRQ_GPIO_Port GPIOF
#define SPI2_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define CCD_CLK_Pin GPIO_PIN_3
#define CCD_CLK_GPIO_Port GPIOC
#define CCD_SI_Pin GPIO_PIN_5
#define CCD_SI_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

extern float v_BL;//左后轮�?�度 cm/s
extern float v_BR;//右后轮�?�度 cm/s
extern float v_C;//车整体�?�度 cm/s
extern float TotalDistance;//小车从某时刻�??????????????始的总距�?????????????? cm
extern uint16_t T_velocity;//测�?�周�??????????????(PID调控周期)，单位：ms
extern uint8_t distance_flag;//�??????????????始累积距离标�??????????????
extern uint8_t MoveFlag;
extern uint16_t T_location;
extern uint8_t distance_flag;//�??????????????始累积距离标�??????????????
extern uint8_t MoveFlag;
extern uint8_t Mode;//模式
extern uint8_t aTxBuffer[];
extern uint8_t aRxBuffer[];
extern uint8_t DCStopFlag;
extern uint8_t TurnFlag;
extern uint32_t TurnCnt;
extern uint32_t TurnPeriod;
extern uint8_t DistanceFlag;
extern uint8_t StraightStopFlag;
extern float TargetDistance;
extern uint16_t ADV[128];
extern uint16_t filtered_ADV[128];//滤波后的像素数组
extern int16_t DxMax;
extern int16_t DxMin;//�?�?/�?小像素差分�??
extern int16_t dX[188];//像素差分数组
extern uint16_t MaxIdx;
extern uint16_t MinIdx;
extern uint16_t TargetIdx;//像素跳变点索引，目标中心点索�?
extern uint8_t median_filter;//是否�?启中值滤�?

void clip(float* val,float min,float max);//限幅函数
void Velocity_Plot(void);//绘制速度波形
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
