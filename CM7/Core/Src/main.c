/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DC.h"
#include "Velocity_PID.h"
#include "dwt_delay.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_LEN 20
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float v_BL;//左后轮�?�度 cm/s
float v_BR;//右后轮�?�度 cm/s
float v_C;//车整体�?�度 cm/s
float Totaldistance;//小车从某时刻�????????????????????????始的总距�???????????????????????? cm
uint16_t T_velocity=20;//测�?�周�????????????????????????(PID调控周期)，单位：ms
uint16_t T_location=100;
uint8_t distance_flag=0;//�????????????????????????始累积距离标�????????????????????????
uint8_t MoveFlag=0;
uint8_t Mode=0;//0-待机模式�???????????????????1-单车模式�???????????????????2-双车模式1 3-双车模式2
char Dir='n';//方向
char Pos='0';//位置
extern Angle_PID_Struct Angle_PID;//转向pid结构�????????????????
extern Location_PID_Struct Location_PID;
extern BL_Velocity_PID_Struct BL_Velocity_PID;
extern BR_Velocity_PID_Struct BR_Velocity_PID;

//电机测试
uint8_t VelocityMeasureFlag=0;
//计时�????????????????????
uint32_t Cnt_1ms=0;

//串口�??????????????????
//Touch Pannel communication define
uint8_t StartSTR[3]={0xff,0xff,0xff};
uint8_t EndSTR[3]={0xff,0xff,0xff};
uint8_t HeaderTxBuffer1[] = "cls BLACK";   //清屏命令
uint8_t HeaderTxBuffer2[] = "page cube_aigc";  //跳转页面命令
uint8_t HeaderTxBuffer3[] = "n0.val=";         //变量赋�?�命�??????????????????
uint8_t HeaderTxBuffer4[20];  //white line
uint8_t HeaderTxBuffer5[] = "add 1,2,";     //real time data
uint8_t HeaderTxBuffer6[20];   //red line
uint8_t HeaderTxBuffer7[20];    //green line

/* Buffer used for data reception */
//uint8_t aRxBuffer[16]="";
uint8_t RxBuffer[16];
uint8_t Touch_pannel_Uart2_RxBuffer[MAX_LEN];
uint8_t Touch_pannel_receive_len = 0;
uint8_t Touch_pannel_receive_completed = 0;
uint8_t Touch_pannel_receive = 0;
uint8_t Touch_pannel_data_receive_start = 0;
uint8_t RxBuffer1[16];

//接收树莓派字符串
char PiRxStrBuf[128];//接收字符串缓冲区

//树莓派识别数�??????????????????
uint8_t SingleNum=0;
uint8_t LeftNum=0;
uint8_t RightNum=0;


uint8_t PiRxCharIdx=0;//接收字符位置索引
uint8_t PiRxChar;//接收的字�??????????????????
uint8_t PiRxStrFlag;//接收字符串标�??????????????????

//药房�??????????????????
uint8_t House=0;

//距离变量
uint8_t distance1=85;//OA
uint8_t distance2=35;//A1
uint8_t distance3=60;//AB
uint8_t distance4=30;//BC
uint8_t distance5=60;//CD
uint8_t distance6=30;//DE
uint8_t distance7=60;//EF
uint8_t distance8=30;//FG
uint8_t distance9=35;//8I

//药物�??????????????????�??????????????????
uint8_t Medicine_Flag=0;//药物装下1,药物卸下2

//车索�??????????????????(1 / 2)
uint8_t car_index=1;

//车返回标�??????????????????
uint8_t ReturnFlag=0;

uint16_t freq=10;//无线通信频率

//调试阶段
char TestStage;
//PID调控�??????????????????启标�??????????????????
/*
uint8_t AnglePID_Flag=1;
uint8_t VelocityPID_Flag=1;
uint8_t LocationPID_Flag=1;
*/

//单纯测试转向�??????????????????
/*
uint8_t AnglePID_Flag=1;
uint8_t VelocityPID_Flag=0;
uint8_t LocationPID_Flag=0;
char TestStage='A';
*/

//单纯测试速度�??????????????????
/*
uint8_t AnglePID_Flag=0;
uint8_t VelocityPID_Flag=1;
uint8_t LocationPID_Flag=0;
char TestStage='V';
*/

//测试速度�??????????????????+转向�??????????????????
/*
uint8_t AnglePID_Flag=1;
uint8_t VelocityPID_Flag=1;
uint8_t LocationPID_Flag=0;
char TestStage='N';
*/

//测试位置+速度�??????????????????+转向�??????????????????
/*
uint8_t AnglePID_Flag=1;
uint8_t VelocityPID_Flag=1;
uint8_t LocationPID_Flag=0;
char TestStage='M';

*/

//无线发�??/接收缓冲�????????????????

//编码器测�??????????????
//TestStage='E';
uint32_t v_cnt=0;//测�?�计数器
float v_BL_Avg;//左轮平均速度
float v_BR_Avg;//右轮平均速度

/*串口测试*/


//速度pid测试
//TestStage='V';
uint8_t Velocity_PID_UpdateFlag=0;
char VelocityStr[20]={};


char NrfTxBuf[32] = {0}; //无线发�?�数据缓冲区
char NrfRxBuf[32]={0};                 //无线接收数据缓冲�???????????????
uint8_t NrfRxFlag=0; //无线接收中断
uint8_t ReceiveHelloFlag=0;

//串口�???????
uint8_t Velocity_Plot_Indicate=0;

//CCD
volatile uint16_t CCD_ADV[128];//CCD 128个像素�??
volatile uint8_t CCD_Cnt=0;//CCD曝光时间计数�???????
volatile uint8_t CCD_Period=10;//CCD曝光时间（ms�???????
volatile uint8_t CCD_ReadFlag=0;//主程序查看CCD值标�???????
volatile uint8_t CCD_ReadCnt;//主程序查看CCD值计数器
volatile uint8_t CCD_ReadPeriod;//主程序查看CCD周期
float actual_Delay=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void MoveTrack1(void);//运动轨迹1：A-O-B
void MoveTrack2(void);//运动轨迹2：A-O-C
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 无线模块接收回调函数
void myRxCallback(char* data) {
	strncpy(NrfRxBuf, data, 32);//拷贝字符串到接收数据缓冲�???????????????
    NrfRxFlag=1;//接受到字符串标志�???????????????1
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  DC_Init();//编码电机初始�???????
  Velocity_PID_Init();//速度pid初始�???????
  DWT_Init();//延时单元初始�???????


//串口测试
  HAL_UART_Receive_IT(&hlpuart1,&PiRxChar,1);//使能接收中断
  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"hi1\r\n",strlen("hi1\r\n"),HAL_MAX_DELAY);
  uint8_t lock=0;
  uint8_t Movelock=0;
  //点击测试
 //无线模块初始�???????????????
  // 1. �???????????????测NRF24L01是否存在
  //while(NRF24L01_Check() != 0) {}
 //SetVelocity(0.1,0.1);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  // 2. 初始化NRF24L01并设置接收回�???????????????
  //Set_RxCallback(myRxCallback);
  //NRF24L01_Init();  // 初始化并启用接收中断
  //转向环pid测试
  //if(TestStage=='A')
  //{
	//SetVelocity(-0.3,-0.3);//基准速度启动
 // }
  //速度pid测试
  SetVelocity(-0.2,-0.2);//启动电机
 // Set_TargetVelocity(-20,-20);//设置pid目标速度20


  /*延时测试
	uint32_t CYCLES_PER_US= SystemCoreClock / 1000000;
	uint32_t start = DWT->CYCCNT;
    DWT_Delay_us(100);
   actual_Delay=(float)(DWT->CYCCNT - start) / CYCLES_PER_US;  // 返回实际延时
	char delay_str[32];
	snprintf(delay_str, sizeof(delay_str), "delay:%.2f", actual_Delay);
	HAL_UART_Transmit(&hlpuart1,delay_str,strlen(delay_str),HAL_MAX_DELAY);
*/


  //使能串口2中断

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*if(TestStage=='U')//串口通讯测试
	  {
	  if(ReceiveHelloFlag==1)
	  {
		  HAL_UART_Transmit(&hlpuart1,"hi5\r\n",strlen("hi5\r\n"),HAL_MAX_DELAY);
		  ReceiveHelloFlag=0;
	  }
	  }*/
	  //查看CCD


	  if(TestStage=='V')
	  {
		  if(Velocity_PID_UpdateFlag==1)
	   {
			  Velocity_PID_UpdateFlag=0;
			  snprintf(VelocityStr,sizeof(VelocityStr),"%.2f,%.2f\r\n",v_BR,BR_Velocity_PID.TargetVelocity);//串口发�?�，绘制当前左轮速度和目标�?�度波形
			  HAL_UART_Transmit(&hlpuart1,VelocityStr,strlen(VelocityStr),HAL_MAX_DELAY);
	   }

	  }
	/*地图(数字1~8代表病房位置�??????????????????0是药房，字母代表交叉处，（字母）表示数字识别�??????????????????)
	 *   7                         8
	 *   |            E            |
	 * G |(F)- - - - - - - - - -(H)| I
	 *   |           |(D)          |
	 *   5           |             6
	 *               |
	 *               | C
     *     3 - - - - - - - - - 4
	 *               |(B)
	 *               |
	 *               |
	 *               |
	 *     1 - - - - - - - - - 2
	 *               |A
	 *               |
	 *               |
	 *               |
	 *               0
	 *
	 * */
   if(TestStage=='0')
   {
	   if(House!=0)
	   {
		//基础部分:单车模式
		switch(Pos)
		{
		  //药房
		  case '0':
			  if(Dir=='n')//药房出发
			  {
				  if(Medicine_Flag==1)//�??????????????????测到药物装上，开始运�??????????????????,直走到�?�A�??????????????????
				  {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance1;
					 Medicine_Flag=0;
				  }

				 if(MoveFlag==0)//到达'A'
				 {
					Pos='A';
				 }
			  }
			  else if(Dir=='s')//返回药房
			  {
				  Dir='n';
				  House=0;
				  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Green Light On",strlen("Green Light On"),HAL_MAX_DELAY);//点亮绿灯
				  Mode=0;
			  }
			  break;
		  case '1':
			  if(Dir=='l')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(1,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='r';
				 }
			  }
			  else if(Dir=='r')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance2;//直走到A
					 lock=1;
				 }
				 if(MoveFlag==0)//到达A
				 {
					Pos='A';
					lock=0;
				 }
			  }

			  break;
		  case '2':
			  if(Dir=='r')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='l';
				 }
			  }
			  else if(Dir=='l')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance2;//直走到A
					 lock=1;
				 }
				 if(MoveFlag==0)//到达A
				 {
					Pos='A';
					lock=0;
				 }
			  }
			  break;
		  case '3':
			  if(Dir=='l')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(1,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='r';
				 }
			  }
			  else if(Dir=='r')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance2;//直走到C
					 lock=1;
				 }
				 if(MoveFlag==0)//到达C
				 {
					Pos='C';
					lock=0;
				 }
			  }
			  break;
		  case '4':
			  if(Dir=='r')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='l';
				 }
			  }
			  else if(Dir=='l')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance2;//直走到C
					 lock=1;
				 }
				 if(MoveFlag==0)//到达C
				 {
					Pos='C';
					lock=0;
				 }
			  }
			  break;
		  case '5':
			  if(Dir=='s')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='n';
				 }
			  }
			  else if(Dir=='n')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance9;//直走到G
					 lock=1;
				 }
				 if(MoveFlag==0)//到达F
				 {
					Pos='G';
					lock=0;
				 }
			  }
			  break;
		  case '6':
			  if(Dir=='s')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='n';
				 }
			  }
			  else if(Dir=='n')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance9;//直走到I
					 lock=1;
				 }
				 if(MoveFlag==0)//到达I
				 {
					Pos='I';
					lock=0;
				 }
			  }
			  break;
		  case '7':
			  if(Dir=='n')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='s';
				 }
			  }
			  else if(Dir=='s')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance9;//直走到G
					 lock=1;
				 }
				 if(MoveFlag==0)//到达F
				 {
					Pos='G';
					lock=0;
				 }
			  }
			  break;
		  case '8':
			  if(Dir=='n')
			  {
				 if(Medicine_Flag==2)//�??????????????????测到药物卸下，回�??????????????????180�??????????????????
				 {
					 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
					 Turn(0,180,1);
					 ReturnFlag=1;
					 Medicine_Flag=0;
					 Dir='s';
				 }
			  }
			  else if(Dir=='s')
			  {
				  if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance9;//直走到I
					 lock=1;
				 }
				 if(MoveFlag==0)//到达I
				 {
					Pos='I';
					lock=0;
				 }
			  }
			  break;

		  //第一交叉�??????????????????
		  case 'A':
			 if(Dir=='n')
			 {

				 //病房1
				 if(House==1)
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='l';
				 }
				 //病房2
				 else if(House==2)
				 {
					Turn(1,90,0.5);//右转90�??????????????????
					Dir='r';
				 }
				 //其他病房
				 else {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance3;//直走到B
						 lock=1;
					 }
					 if(MoveFlag==0)//到达B
					 {
						Pos='C';
						lock=0;
					 }

			   }
			 }
			 else if(Dir=='s') // //直走�??????????????????'0'
			 {
				 if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance1;//直走到B
					 lock=1;
				 }
				 if(MoveFlag==0)//到达0
				 {
					Pos='0';
					lock=0;
				 }

			 }
			 else if(Dir=='l')
			 {
				 if(ReturnFlag==1)//返回
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='s';
					ReturnFlag=0;
				 }
				 else //前进，到�??????????????????1
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance2;//直走到B
						 lock=1;
					 }
					 if(MoveFlag==0)//到达1
					 {
						Pos='1';
						HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light On",strlen("Red Light On"),HAL_MAX_DELAY);//点亮红灯
						lock=0;
					 }
				 }

			 }
			 else if(Dir=='r')
			  {
				 if(ReturnFlag==1) //返回
				 {
					 Turn(0,90,0.5);//右转90�??????????????????
					 Dir='s';
					 ReturnFlag=0;
				 }
				 else //前进
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance2;//直走到B
						 lock=1;
					 }
					 if(MoveFlag==0)//到达2
					 {
						Pos='2';
						HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light On",strlen("Red Light On"),HAL_MAX_DELAY);//点亮红灯
						lock=0;
					 }
				 }

			  }

			  break;
		  case 'B':
			  if(Dir=='n')
			  {
				  if(LeftNum==0 && RightNum==0 && lock==0)//发�?�命令给树莓派识别数�??????????????????
				  {
					  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Recognize Two Numbers",strlen("Recognize Two Numbers"),HAL_MAX_DELAY);//点亮红灯
					  lock=1;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==1)//识别数字完成，直走至C
				  {
					  MoveFlag=1;
					  Cnt_1ms=0;
					  Location_PID.TargetLocation=distance4;//直走到C
					  lock=0;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==0)
				  {
					  if(MoveFlag==0)//到达C
					  {
						  Pos='C';
					  }
				  }
			  }
			  break;
		  case 'C':
			  if(Dir=='n')
			 {
				 if(House==LeftNum)//目标病房号等于B左边数字,左转
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='l';
				 }
				 else if(House==RightNum)//目标病房号等于B右边数字,右转
				 {
					Turn(1,90,0.5);//右转90�??????????????????
					Dir='r';
				 }
				 //其他病房
				 else { //目标病房号在前面，继续前�??????????????????
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance5;//直走到D
						 lock=1;
					 }
					 if(MoveFlag==0)//到达D
					 {
						Pos='D';
						lock=0;
					 }

			   }
			 }
			   else if(Dir=='s') //直走�??????????????????0
			   {
				 if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance4+distance3+distance1;//直走到B
					 lock=1;
				 }
				 if(MoveFlag==0)//到达0
				 {
					Pos='0';
					lock=0;
				 }

			   }
			   else if(Dir=='l')
			   {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(0,90,0.5);//左转90�??????????????????
					 ReturnFlag=0;
					 Dir='s';
				 }

				 else//前进
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance2;//直走到B
						 lock=1;
					 }
					 if(MoveFlag==0)//到达3
					 {
						Pos='3';
						HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light On",strlen("Red Light On"),HAL_MAX_DELAY);//点亮红灯
						lock=0;
					 }
				 }
			  }
			   else if(Dir=='r')
			  {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(1,90,0.5);//右转90�??????????????????
					 ReturnFlag=0;
					 Dir='s';
				 }
				 else //前进
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance2;//直走�??????????????????4
						 lock=1;
					 }
					 if(MoveFlag==0)//到达4
					 {
						Pos='4';
						HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light On",strlen("Red Light On"),HAL_MAX_DELAY);//点亮红灯
						lock=0;
					 }
				 }

			  }

			  break;
		  case 'D':
			  if(Dir=='n')
			  {
				  if(LeftNum==0 && RightNum==0 && lock==0)//发�?�命令给树莓派识别数�??????????????????
				  {
					  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Recognize Two Numbers",strlen("Recognize Two Numbers"),HAL_MAX_DELAY);//点亮红灯
					  lock=1;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==1)//识别数字完成，直走至E
				  {
					  MoveFlag=1;
					  Cnt_1ms=0;
					  Location_PID.TargetLocation=distance6;//直走到E
					  lock=0;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==0)
				  {
					  if(MoveFlag==0)//到达E
					  {
						  Pos='E';
					  }
				  }
			  }
			  break;
		  case 'E':
			   if(Dir=='n')
			  {
				 if(House==LeftNum)//目标病房号等于D左边数字,左转
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='l';
				 }
				 else if(House==RightNum)//目标病房号等于D右边数字,右转
				 {
					Turn(1,90,0.5);//右转90�??????????????????
					Dir='r';
				 }
			  }
			   else if(Dir=='s') //直走�??????????????????0
			   {
				 if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance6+distance5+distance4+distance3+distance1;//直走�??????????????????0
					 lock=1;
				 }
				 if(MoveFlag==0)//到达0
				 {
					Pos='0';
					lock=0;
				 }

			   }
			   else if(Dir=='l')
			   {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(0,90,0.5);//左转90�??????????????????
					 ReturnFlag=0;
					 Dir='s';
				 }
				 else//前进
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance7;//直走到F
						 lock=1;
					 }
					 if(MoveFlag==0)//到达F
					 {
						Pos='F';
						lock=0;
					 }
				 }

			   }
			   else if(Dir=='r') //直走到�?�H�??????????????????
			  {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(1,90,0.5);//右转90�??????????????????
					 ReturnFlag=0;
					 Dir='s';
				 }
				 else
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance7;//直走到B
						 lock=1;
					 }
					 if(MoveFlag==0)//到达H
					 {
						Pos='H';
						lock=0;
					 }
				 }
			  }
			  break;
		  case 'F':
			  if(Dir=='l')
			  {
				  if(LeftNum==0 && RightNum==0 && lock==0)//发�?�命令给树莓派识别数�??????????????????
				  {
					  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Recognize Two Numbers",strlen("Recognize Two Numbers"),HAL_MAX_DELAY);//点亮红灯
					  lock=1;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==1)//识别数字完成，直走至G
				  {
					  MoveFlag=1;
					  Cnt_1ms=0;
					  Location_PID.TargetLocation=distance8;//直走到G
					  lock=0;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==0)
				  {
					  if(MoveFlag==0)//到达G
					  {
						  Pos='G';
					  }
				  }
			  }
			  break;
		  case 'G':
			  if(Dir=='l')
			  {
				 if(House==LeftNum)//目标病房号等于F左边数字,左转
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='s';
				 }
				 else if(House==RightNum)//目标病房号等于F右边数字,右转
				 {
					Turn(1,90,0.5);//右转90�??????????????????
					Dir='n';
				 }
			  }
			   else if(Dir=='r') //直走到E
			   {
				 if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance8+distance7;//直走到E
					 lock=1;
				 }
				 if(MoveFlag==0)//到达E
				 {
					Pos='E';
					lock=0;
				 }

			   }
			   else if(Dir=='n')
			   {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(1,90,0.5);//右转90�??????????????????
					 ReturnFlag=0;
					 Dir='r';
				 }
				 else
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance9;//直走�??????????????????7
						 lock=1;
					 }
					 if(MoveFlag==0)//到达7
					 {
						Pos='7';
						lock=0;
					 }
				 }

			   }
			   else if(Dir=='s')
			  {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(0,90,0.5);//左转90�??????????????????
					 ReturnFlag=0;
					 Dir='r';
				 }
				 else
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance9;//直走�??????????????????5
						 lock=1;
					 }
					 if(MoveFlag==0)//到达5
					 {
						Pos='5';
						lock=0;
					 }
				 }

			  }
			  break;
		  case 'H':
			  if(Dir=='r')
			  {
				  if(LeftNum==0 && RightNum==0 && lock==0)//发�?�命令给树莓派识别数�??????????????????
				  {
					  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Recognize Two Numbers",strlen("Recognize Two Numbers"),HAL_MAX_DELAY);//点亮红灯
					  lock=1;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==1)//识别数字完成，直走至G
				  {
					  MoveFlag=1;
					  Cnt_1ms=0;
					  Location_PID.TargetLocation=distance8;//直走到I
					  lock=0;
				  }
				  else if(LeftNum!=0 && RightNum!=0 && lock==0)
				  {
					  if(MoveFlag==0)//到达G
					  {
						  Pos='I';
					  }
				  }
			  }
			  break;
		  case 'I':
			  if(Dir=='l')
			  {
				 if(House==LeftNum)//目标病房号等于H左边数字,左转
				 {
					Turn(0,90,0.5);//左转90�??????????????????
					Dir='n';
				 }
				 else if(House==RightNum)//目标病房号等于H右边数字,右转
				 {
					Turn(1,90,0.5);//右转90�??????????????????
					Dir='s';
				 }
			  }
			   else if(Dir=='l') //直走到E
			   {
				 if(lock==0)
				 {
					 MoveFlag=1;
					 Cnt_1ms=0;
					 Location_PID.TargetLocation=distance8+distance7;//直走到E
					 lock=1;
				 }
				 if(MoveFlag==0)//到达E
				 {
					Pos='E';
					lock=0;
				 }

			   }
			   else if(Dir=='n')
			   {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(0,90,0.5);//左转90�??????????????????
					 ReturnFlag=0;
					 Dir='l';
				 }
				 else
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance9;//直走�??????????????????8
						 lock=1;
					 }
					 if(MoveFlag==0)//到达8
					 {
						Pos='8';
						lock=0;
					 }
				 }

			   }
			   else if(Dir=='s')
			  {
				 if(ReturnFlag==1)//返回
				 {
					 Turn(1,90,0.5);//右转90�??????????????????
					 ReturnFlag=0;
					 Dir='l';
				 }
				 else
				 {
					 if(lock==0)
					 {
						 MoveFlag=1;
						 Cnt_1ms=0;
						 Location_PID.TargetLocation=distance9;//直走�??????????????????6
						 lock=1;
					 }
					 if(MoveFlag==0)//到达6
					 {
						Pos='6';
						lock=0;
					 }
				 }
			  }
			  break;
		  default:
			  break;


		}

       /*
         if(MoveFlag==1 && Movelock==0)
	   {
		   SetVelocity(0.1,0.1);//起始低�??
		   Movelock=1;
	   }
	   if(MoveFlag==0)
	   {
		   Movelock=0;
	   }
	   */
   }
	   /*树莓派命令串口响�??????????????????*/
	   	if(PiRxStrFlag==1)
	   	{
	   			uint8_t BL_SetVelocity=0;//通过串口命令设置的左轮目标�?�度
	   			uint8_t BR_SetVelocity=0;//通过串口命令设置的右轮目标�?�度
	   			uint8_t Kp=0;//通过串口命令设置的kp
	   			uint8_t Ki=0;//通过串口命令设置的Ki
	   			uint8_t Kd=0;//通过串口命令设置的Kd
	   			//识别单个数字
	   			if(sscanf((const char *)&PiRxStrBuf,"Recognize One Number:%d",SingleNum)==1)
	   			{
	   				House=SingleNum;
	   			}

	   			//识别两个数字
	   			else if(sscanf((const char *)&PiRxStrBuf,"Recognize Two Numbers Left:%d,Right:%d",LeftNum,RightNum)==2)
	   			{

	   			}

	   			//接收到检测到药物的命令后，小车开始运�??????????????????
	   			else if(strcmp(&PiRxStrBuf,"Detect Medicine On")==0)
	   			{
	   				Medicine_Flag=1;
	   			}

	   			else if(strcmp(&PiRxStrBuf,"Detect Medicine Off")==0)
	   			{
	   				 Medicine_Flag=2;
	   			}
	   			else if(strcmp(&PiRxStrBuf,"hi4")==0)
	   			{
	   				 ReceiveHelloFlag=1;
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBL V %f",BL_SetVelocity)==0)//设置左轮目标速度
	   			{
	   				Set_BL_TargetVelocity(BL_SetVelocity);
	   				BL_Velocity_PID_Reset();
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBR V %f",BR_SetVelocity)==0)//设置右轮目标速度
	   			{
	   				Set_BR_TargetVelocity(BR_SetVelocity);
	   				BR_Velocity_PID_Reset();
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBL Kp %f",Kp)==0)//设置左轮Kp
	   			{
	   				Set_BL_Kp(Kp);
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBL Ki %f",Ki)==0)//设置左轮Ki
	   			{
	   				Set_BL_Ki(Ki);
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBL Kd %f",Kd)==0)//设置左轮Kd
	   			{
	   				Set_BL_Kd(Kd);
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBR Kp %f",Kp)==0)//设置右轮Kp
	   			{
	   				Set_BR_Kp(Kp);
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBR Ki %f",Ki)==0)//设置右轮Ki
	   			{
	   				Set_BR_Ki(Ki);
	   			}
	   			else if(sscanf(&PiRxStrBuf,"SetBR Kd %f",Kd)==0)//设置右轮Kd
	   			{
	   				Set_BR_Kd(Kd);
	   			}
	   			else if(strcmp(&PiRxStrBuf,"Stop")==0)//车辆停转
	   			{
	   				DC_Stop();
	   			}
	   			PiRxStrFlag=0;
	   		}
	  }

   /*串口屏命令响�??????????????????*/
   	if( Touch_pannel_receive_completed ==1)
   	{
   		switch(Touch_pannel_Uart2_RxBuffer[1])
   		{
   	      //模式切换:单车模式
   		   case 0x01:
   			  Mode=1;
   			  //发�?�命令给树莓派，准备识别�??????????????????个数�??????????????????
   			  HAL_UART_Transmit(&hlpuart1,"Recognize One Number",strlen("Recognize One Number"),HAL_MAX_DELAY);
   			  HAL_UART_Transmit(&hlpuart1,"Green Light Off",strlen("Green Light Off"),HAL_MAX_DELAY);//熄灭上一次任务完成后点亮的绿�??????????????????
   			  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
   			  break;
   	      //模式切换:双车模式1（拓展题1�??????????????????
   		   case 0x02:
   			  Mode=2;
   			  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
   			  break;
   	     //模式切换:双车模式2（拓展题2�??????????????????
			   case 0x03:
				  Mode=3;
				  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
		     //车前进（有�?�度pid�???????
			   case 0x11:
				  SetVelocity(0.2,0.2);//启动电机
				  Set_TargetVelocity(20,20);//设置pid目标速度20
				  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
			 //车后�???????（有速度pid�???????
			   case 0x12:
				   SetVelocity(-0.2,-0.2);//启动电机
				   Set_TargetVelocity(-20,-20);//设置pid目标速度20
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
			//车停�???????
			   case 0x13:
				   DC_Stop();
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
		  //拖动滑钮，调整pid目标速度
			   case 0x14:
				   float TargetSpeed=V_MIN+(V_MAX-V_MIN)*Touch_pannel_Uart2_RxBuffer[2]/100.0;
				   Set_TargetVelocity(TargetSpeed,TargetSpeed);
			       Touch_pannel_Uart2_RxBuffer[1] = 0x0;
			       break;
		   //绘制速度波形
			   case 0x15:
				   Velocity_Plot_Indicate=1;
			       Touch_pannel_Uart2_RxBuffer[1] = 0x0;
			       break;
		   //�???????出�?�度波形界面
			   case 0x16:
				   Velocity_Plot_Indicate=0;
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;


   		   default:

   			 break;
   		}
   		Touch_pannel_receive_completed =0;
   	}


	if(Velocity_Plot_Indicate==1)
   {
	 Velocity_Plot();
   }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//1ms定时中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if(htim->Instance == TIM5)
  {





      //速度pid调控周期�????????????????????20ms
      if(MoveFlag==1)
      {
    	  Cnt_1ms++;
		  if(Cnt_1ms%T_velocity==0)
		  {
			  GetVelocity();//更新左右轮转�???????
			  //Velocity_Update();//速度PID控制
			 // Velocity_PID_UpdateFlag=1;
			  /*占空�???????-电机转�?�关系测�???????
			  v_cnt++;
			  GetVelocity();
			  if(v_cnt>250 && v_cnt<750)
			  {
			    v_BR_Avg=(v_BR_Avg*(v_cnt-251)+v_BR)/(v_cnt-250);
			    v_BL_Avg=(v_BL_Avg*(v_cnt-251)+v_BL)/(v_cnt-250);
			  }
			  */
			  //Location_PID.CurLocation=Totaldistance;
              //Velocity_PID_Control();
		  }

		  //位置pid调控周期100ms
		  if(Cnt_1ms%T_location==0)
		  {
			  //Location_PID_Control();
			  //Velocity_PID.TargetVelocity=Location_PID.Output;
		  }

      }

  }
}
//串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//串口�??????????????
	if ((huart->Instance == USART2)&&(HAL_UART_Receive_IT(huart, &Touch_pannel_receive, 1) == HAL_OK))
	{
	      if((char)Touch_pannel_receive == 0x5A)
	        {
	          Touch_pannel_data_receive_start = 0x1;
	        }

	      if ((Touch_pannel_receive_len < MAX_LEN)&&(Touch_pannel_data_receive_start == 0x1))
	        {
	          Touch_pannel_Uart2_RxBuffer[Touch_pannel_receive_len] = (char)Touch_pannel_receive;
	          if ((char)Touch_pannel_receive == 0xA5)
	          {
	            Touch_pannel_receive_completed = 1;
	            Touch_pannel_receive_len = 0;
	            Touch_pannel_data_receive_start = 0x0;
	          }
	          else
	          {
	            Touch_pannel_receive_len += 1;
	          }
	        }
	}
	//和stm32通信（与树莓派间接�?�信�??????????????
	if(huart==&hlpuart1)
	{
		if(PiRxChar=='\r')
		{

		}
		else if(PiRxChar=='\n')
		{
			PiRxStrBuf[PiRxCharIdx++]='\0';
			PiRxCharIdx=0;
			PiRxStrFlag=1;
		}
		else
		{
			PiRxStrBuf[PiRxCharIdx++]=PiRxChar;
		}
		HAL_UART_Receive_IT(&hlpuart1,&PiRxChar,1);//重新使能接收中断
	}
}


void clip(float* val,float min,float max)//限幅函数
{
	if(val>0)
	{
		*val=(*val>min)?*val:min;
		*val=(*val<max)?*val:max;
	}
	else
	{
		*val=(*val>-max)?*val:-max;
		*val=(*val<-min)?*val:-min;
	}
}

//绘制速度波形
void Velocity_Plot(void)
{
	uint8_t speed[4];
    uint8_t tmp0,tmp1,tmp2,tmp3,tmp4,tmp5,tmp6;
	uint16_t tempSpeed= BL_Velocity_PID.CurVelocity;
	tmp0 = tempSpeed%1000;
	tmp1 = tempSpeed/1000+0x30;  //千位�???????
	tmp2 = tmp0%100;
	tmp3 = tmp0/100+0x30;  //百位�???????
	tmp4 = tmp2%10;
	tmp5 = tmp2/10+0x30;  //十位�???????
	tmp6 = tmp4+0x30;  //个位�???????

	speed[0] = tmp3;
	speed[1] = tmp5;
	speed[2] = tmp6;

	HAL_UART_Transmit(&huart2,StartSTR,3,10);
	HAL_UART_Transmit(&huart2,HeaderTxBuffer5,sizeof(HeaderTxBuffer5)-1,10);
	HAL_UART_Transmit(&huart2,speed,3,10);
	HAL_UART_Transmit(&huart2,EndSTR,3,10);

}



/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
