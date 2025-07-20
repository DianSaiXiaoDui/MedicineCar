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
#include "Angle_PID.h"
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
enum Place {start,pharmacy,ward,cross1,cross2};
enum Place place = pharmacy;
int ActionLog[20] = {}; // 0代表前进, 1代表左转, -1代表右转
int ActionLogTop = -1;
uint8_t actionLock = 0;
float v_BL;//左后轮�?�度 cm/s
float v_BR;//右后轮�?�度 cm/s
float v_C;//车整体�?�度 cm/s
float TotalDistance;//小车从某时刻�?????????????????????????始的总距�????????????????????????? cm
uint16_t T_velocity=20;//测�?�周�?????????????????????????(PID调控周期)，单位：ms
uint16_t T_location=100;
uint8_t distance_flag=0;//�?????????????????????????始累积距离标�?????????????????????????
uint8_t MoveFlag=0;
uint8_t toWard = 1;
uint8_t cross_detected = 0;
uint8_t block_detected = 0;
uint8_t medicine_detected = 0;
uint16_t regularVelocity = 20;
uint16_t toBlockTime = 500;
uint16_t toCrossTime = 500;
char target = ' ';
char Dir='n';//方向
char Pos='S';//位置
char action[20] = "stop";
char digitDetected[20]= "";
extern Angle_PID_Struct Angle_PID;//转向pid结构�?????????????????
extern BL_Velocity_PID_Struct BL_Velocity_PID;
extern BR_Velocity_PID_Struct BR_Velocity_PID;

//电机测试
uint8_t VelocityMeasureFlag=0;
//计时�?????????????????????
uint32_t Cnt_1ms=0;

//串口�???????????????????
//Touch Pannel communication define
uint8_t StartSTR[3]={0xff,0xff,0xff};
uint8_t EndSTR[3]={0xff,0xff,0xff};
uint8_t HeaderTxBuffer1[] = "cls BLACK";   //清屏命令
uint8_t HeaderTxBuffer2[] = "page cube_aigc";  //跳转页面命令
uint8_t HeaderTxBuffer3[] = "n0.val=";         //变量赋�?�命�???????????????????
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

//药物�???????????????????�???????????????????
uint8_t Medicine_Flag=0;//药物装下1,药物卸下2

//车索�???????????????????(1 / 2)
uint8_t car_index=1;

//车返回标�???????????????????
uint8_t ReturnFlag=0;

uint16_t freq=10;//无线通信频率

//调试阶段
char TestStage;


uint32_t v_cnt=0;//测�?�计数器
float v_BL_Avg;//左轮平均速度
float v_BR_Avg;//右轮平均速度

/*串口测试*/


//速度pid测试
//TestStage='V';
uint8_t Velocity_PID_UpdateFlag=0;
char VelocityStr[20]={};
uint8_t DCStopFlag=0;
uint8_t TurnFlag=0;
uint32_t TurnCnt=0;
uint32_t WaitCnt = 0;
uint32_t TurnPeriod=0;
uint32_t WaitPeriod = 10;
uint8_t TurnStopFlag=0;
uint8_t StopFlag = 0;
uint8_t WaitFlag = 0; // 给与一定数字识别时间
uint8_t OpenFlag = 0; // 给与一定开环往前走
uint8_t OpenDis = 10;
uint8_t LoopStart = 0; // 是否是一次新的开环走+旋转
//uint32_t TurnNinetyPeriod=650;
//uint32_t TurnBackPeriod=1100;



uint8_t DistanceFlag=0;
float TargetDistance=0;
uint8_t StraightStopFlag=0;

char NrfTxBuf[32] = {0}; //无线发�?�数据缓冲区
char NrfRxBuf[32]={0};                 //无线接收数据缓冲�????????????????
uint8_t NrfRxFlag=0; //无线接收中断
uint8_t ReceiveHelloFlag=0;

uint8_t PiRxStrBuf[128];
uint8_t PiRxCharIdx=0;//接收字符位置索引
uint8_t PiRxChar;//接收的字�???????????????????
uint8_t PiRxStrFlag;//接收字符串标�???????????????????

//串口�????????
uint8_t Velocity_Plot_Indicate=0;

float actual_Delay=0;
int8_t actions[3];
uint8_t action_index = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void MoveTrack1(void);//运动轨迹1：A-O-B
void MoveTrack2(void);//运动轨迹2：A-O-C
uint8_t OpenForward();
uint8_t OpenTurn(uint8_t, uint8_t);
void openLoopTurning(uint8_t clockwise,uint8_t angle);
void openLoopForward(uint8_t forwardVelocity, uint8_t forwardTime);
void Enable_CrossDetected(void);
void Enable_BlockDetected(void);
void Require_Numbers(uint8_t num);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 无线模块接收回调函数
void myRxCallback(char* data) {
	strncpy(NrfRxBuf, data, 32);//拷贝字符串到接收数据缓冲�????????????????
    NrfRxFlag=1;//接受到字符串标志�????????????????1
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
  DC_Init();//编码电机初始�????????
  Velocity_PID_Init();//速度pid初始�????????
  DWT_Init();//延时单元初始�????????

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&RxBuffer, 1);
//串口测试
//  HAL_UART_Receive_IT(&hlpuart1,&PiRxChar,1);//使能接收中断
  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"hi1\r\n",strlen("hi1\r\n"),HAL_MAX_DELAY);
  uint8_t lock=0;
  uint8_t Movelock=0;
  TestStage='V';


  //使能串口2中断
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&RxBuffer, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/*地图(数字1~8代表病房位置�???????????????????0是药房，字母代表交叉处，（字母）表示数字识别�???????????????????)
	 *   7                         8
	 *   |            C            |
	 * D |   - - - - - - - - - -   | E
	 *   |           |             |
	 *   5           |             6
	 *               |
	 *               | B
     *     3 - - - - - - - - - 4
	 *               |
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
		//基础部分运动逻辑状态转换机
		switch(Pos)
		{
		  case 'S':
			  if (target != ' ')
				  Pos = '0';
			  break;
		  //药房
		  case '0':
			  if(Dir=='n')//药房出发
			  {
				  if(medicine_detected==1)//检测到药物被装上
				  {
					  DC_Start(0);
					  medicine_detected=0;//清除药物检测标志
					  Enable_CrossDetected();//等待十字路口检测
				  }
				  if(cross_detected==1)
				  {
					  Pos = 'A';
				      cross_detected=0;
				  }
			  }
			  else if(Dir=='s')//返回药房
			  {
				  Dir='n';
				  Pos = 'S';
				  target = ' ';
				  action_index=-1;
				  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Green Light On",strlen("Green Light On"),HAL_MAX_DELAY);//点亮绿灯
			  }
			  break;
		  case '1':
			  if(Dir == 'w')
			  {
			     if(MoveFlag)
			    	 openLoopForward(regularVelocity,toBlockTime);
			     if(!MoveFlag && medicine_detected==2)
			     {
			    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
			    	 openLoopTurning(1,180);
			    	 toWard = 0;
			    	 medicine_detected=0;//清除药物检测标志
			    	 Dir = 'e';
			     }
			  }
			  else if (Dir == 'e')
			  {
				  if(!MoveFlag)
				  {
					  DC_Start(0);
					  Enable_CrossDetected();
				  }
				  if (cross_detected)
				  {
					  Pos = 'A';
				  }
			  }
			  break;
		  case '2':
			  if(Dir == 'e')
			  {
			     if(MoveFlag)
			    	 openLoopForward(regularVelocity,toBlockTime);
			     if(!MoveFlag && medicine_detected==2)
			     {
			    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
			    	 openLoopTurning(1,180);
			    	 medicine_detected=0;//清除药物检测标志
			    	 toWard = 0;
			    	 Dir = 'w';
			     }
			  }
			  else if (Dir == 'w')
			  {
				  if(!MoveFlag)
				  {
					  DC_Start(0);
					  Enable_CrossDetected();
				  }
				  if (cross_detected)
				  {
					  Pos = 'A';
				  }
			  }
			break;
		  case 'A':
			  if(Dir == 'n')
			  {
				  if (target != '1' &&  target != '2')
				  {
					  Enable_CrossDetected();//等待十字路口检测
					  if(cross_detected)//检测到十字路口
					  {
						  actions[++action_index] = 0;
						  Pos = 'B';
					  }
				  }
				  else if(target =='1')
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 'w';
					  }
				  }
			      else if(target == '2')
			      {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 'e';
					  }
				  }
			  }
			  else if(Dir == 'w')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = '1';
						  block_detected=0;
					  }
				  }
				  else
				  {

				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 'e')
			  {
				  if(toWard)
				  {

					  if(!MoveFlag)
					  {
						DC_Start(0);
						Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = '2';
						  block_detected=0;
					  }
				  }
				  else
				  {

					  openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 's')
			  {
				  if(!MoveFlag)
				  {
					DC_Start(0);
					Enable_BlockDetected();
				  }
				  if(block_detected)
				  {
					  Pos = '0';
					  block_detected=0;
					  action_index--;
				  }
			  }
			  break;
		  case 'm':
			  if (actions[action_index] == -1)
			  {
				  if(Dir == 'w')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 'e';
				     }
				  }
				  else if (Dir == 'e')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'B';
					  }
				  }
			  }
			  else if (actions[action_index] == 1)
			  {
				  if(Dir == 'e')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 'w';
				     }
				  }
				  else if (Dir == 'w')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'B';
					  }
				  }
			  }
			  break;
		  case 'B':
			  if(Dir == 'n')
			  {
				  if (strlen(digitDetected) < 2)
				  {
					  DC_Stop();
					  Require_Numbers(2);
				  }
				  else if (target != digitDetected[0] &&  target != digitDetected[1])
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if(cross_detected)
					  {
						  actions[++action_index] = 0;
						  Pos = 'C';
					  }
				  }
				  else if(target == digitDetected[0])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 'w';
					  }
					  actions[++action_index] = -1;
				  }
				  else if(target == digitDetected[1])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 'e';
					  }
					  actions[++action_index] = 1;
				  }
			  }
			  else if(Dir == 'w')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'm';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 'e')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'm';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 's')
			  {
				  if(!MoveFlag)
				  {
					DC_Start(0);
					Enable_BlockDetected();
				  }
				  if(block_detected)
				  {
					  Pos = '0';
					  block_detected=0;
				  }

			  }
			  break;
		  case 'C':
			  if(Dir == 'n')
			  {
				  if (strlen(digitDetected) < 4)
				  {
					  DC_Stop();
					  Require_Numbers(4);
				  }

				  else if(target == digitDetected[0] || target == digitDetected[1])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 'w';
					  }
					  actions[++action_index] = -1;
				  }
				  else if(target == digitDetected[1] || target==digitDetected[2])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 'e';
					  }
					  actions[++action_index] = 1;
				  }
			  }
			  else if(Dir == 'w')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_CrossDetected();
					  }
					  if(cross_detected)
					  {
						  Pos = 'D';
						  cross_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 'e')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_CrossDetected();
					  }
					  if(cross_detected)
					  {
						  Pos = 'E';
						  cross_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 's')
			  {
				  if(!MoveFlag)
				  {
					DC_Start(0);
					Enable_BlockDetected();
				  }
				  if(block_detected)
				  {
					  Pos = '0';
					  block_detected=0;
				  }
			  }
			  break;
		  case 'D':
			  if(Dir == 'w')
			  {
				  if (strlen(digitDetected) < 2)
				  {
					  DC_Stop();
					  Require_Numbers(2);
				  }

				  else if(target == digitDetected[0])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 's';
					  }
					  actions[++action_index] = -1;
				  }
				  else if(target == digitDetected[1])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 'n';
					  }
					  actions[++action_index] = 1;
				  }
			  }
			  else if(Dir == 's')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'l';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 'n';
					  }
				  }
			  }
			  else if(Dir == 'n')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'l';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 'e')
			  {
				  if(!MoveFlag)
				  {
					DC_Start(0);
					Enable_BlockDetected();
				  }
				  if(block_detected)
				  {
					  Pos = 'C';
					  block_detected=0;
				  }
			  }
			  break;
		  case 'l':
			  if (actions[action_index] == -1)
			  {
				  if(Dir == 's')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 'n';
				     }
				  }
				  else if (Dir == 'n')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'D';
					  }
				  }
			  }
			  else if (actions[action_index] == 1)
			  {
				  if(Dir == 'n')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 's';
				     }
				  }
				  else if (Dir == 's')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'D';
					  }
				  }
			  }
			  break;
		  case 'E':
			  if(Dir == 'e')
			  {
				  if (strlen(digitDetected) < 2)
				  {
					  DC_Stop();
					  Require_Numbers(2);
				  }

				  else if(target == digitDetected[0])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 'n';
					  }
					  actions[++action_index] = -1;
				  }
				  else if(target == digitDetected[1])
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 's';
					  }
					  actions[++action_index] = 1;
				  }
			  }
			  else if(Dir == 'n')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'r';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(-1,90);
						  Dir = 's';
					  }
				  }
			  }
			  else if(Dir == 's')
			  {
				  if(toWard)
				  {
					  if(!MoveFlag)
					  {
					    DC_Start(0);
					    Enable_BlockDetected();
					  }
					  if(block_detected)
					  {
						  Pos = 'r';
						  block_detected=0;
					  }
				  }
				  else
				  {
				      openLoopForward(regularVelocity,toCrossTime);

					  if(!MoveFlag)
					  {
						  openLoopTurning(1,90);
						  Dir = 'n';
					  }
				  }
			  }
			  else if(Dir == 'w')
			  {
				  if(!MoveFlag)
				  {
					DC_Start(0);
					Enable_BlockDetected();
				  }
				  if(block_detected)
				  {
					  Pos = 'C';
					  block_detected=0;
				  }
			  }
			  break;
		  case 'r':
			  if (actions[action_index] == 1)
			  {
				  if(Dir == 's')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 'n';
				     }
				  }
				  else if (Dir == 'n')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'E';
					  }
				  }
			  }
			  else if (actions[action_index] == -1)
			  {
				  if(Dir == 'n')
				  {
				     if(MoveFlag)
				    	 openLoopForward(regularVelocity,toBlockTime);
				     if(!MoveFlag && medicine_detected==2)
				     {
				    	 HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Red Light Off",strlen("Red Light Off"),HAL_MAX_DELAY);//熄灭红灯
				    	 openLoopTurning(1,180);
				    	 medicine_detected=0;//清除药物检测标志
				    	 toWard = 0;
				    	 Dir = 's';
				     }
				  }
				  else if (Dir == 's')
				  {
					  if(!MoveFlag)
					  {
						  DC_Start(0);
						  Enable_CrossDetected();
					  }
					  if (cross_detected)
					  {
						  Pos = 'E';
					  }
				  }
			  }
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



   /*串口屏命令响应*/
   	if( Touch_pannel_receive_completed ==1)
   	{
   		switch(Touch_pannel_Uart2_RxBuffer[1])
   		{
   	      //模式切换:单车模式
   		   case 0x01:
//   			  Mode=1;
   			  //发�?�命令给树莓派，准备识别�???????????????????个数�???????????????????
   			  HAL_UART_Transmit(&hlpuart1,"Recognize One Number",strlen("Recognize One Number"),HAL_MAX_DELAY);
   			  HAL_UART_Transmit(&hlpuart1,"Green Light Off",strlen("Green Light Off"),HAL_MAX_DELAY);//熄灭上一次任务完成后点亮的绿�???????????????????
   			  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
   			  break;
   	      //模式切换:双车模式1（拓展题1�???????????????????
   		   case 0x02:
//   			  Mode=2;
   			  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
   			  break;
   	     //模式切换:双车模式2（拓展题2�???????????????????
			   case 0x03:
//				  Mode=3;
				  Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
		     //车前进（有�?�度pid�????????
			   case 0x11:
				  // DC_Forward(90,0);
                  TestStage='Z';
                  Pos='0';
                  Dir='n';
				  Touch_pannel_Uart2_RxBuffer[1] = 0x0;

				  break;
			 //车后�????????（有速度pid�????????
			   case 0x12:
				   DC_Backward(0,1);
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				  break;
			//车停�????????
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
		   //�????????出�?�度波形界面
			   case 0x16:
				   Velocity_Plot_Indicate=0;
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
			//车左�?
			   case 0x17:
				   openLoopTurning(-1, 90);
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
			//车右�?
			   case 0x18:
				   openLoopTurning(1, 90);
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
			//车掉�?
			   case 0x19:
				   openLoopTurning(1, 180);
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
				//开始执行任务时，向单片机发送识别一个数字的请求
			   case 0x20:
				   Require_Numbers(1);
				   break;
				//模拟药物装上
			   case 0x21:
				   medicine_detected=1;
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
				//模拟药物卸下
			   case 0x22:
				   medicine_detected=2;
				   Touch_pannel_Uart2_RxBuffer[1] = 0x0;
				   break;
   		   default:

   			 break;
   		}
   		Touch_pannel_receive_completed =0;
   	}

	   //与k230通信
	   	if(PiRxStrFlag==1)
	   	{
            uint8_t Num1=0,Num2=0,Num3=0,Num4=0;
            uint8_t TempCx=0;
            //识别到一个数字
			if(sscanf((const char *)&PiRxStrBuf,"detect one number: %d",Num1)==1)
			{
				target='0'+Num1;//记录当前要去的病房号
			}
			//识别到两个数字
			else if(sscanf((const char *)&PiRxStrBuf,"detect two numbers: %d %d",Num1,Num2)==2)
			{
				digitDetected[0]='0'+Num1;
				digitDetected[1]='0'+Num2;
				digitDetected[2]='\0';
			}
            //识别到四个数字
			else if(sscanf((const char *)&PiRxStrBuf,"detect two numbers: %d %d %d %d",Num1,Num2,Num3,Num4)==4)
			{
				digitDetected[0]='0'+Num1;
				digitDetected[1]='0'+Num2;
				digitDetected[2]='0'+Num3;
				digitDetected[3]='0'+Num4;
				digitDetected[4]='\0';
			}
			else if(strcmp(&PiRxStrBuf,"Hello From K230!")==0)//车辆停转
			{
			  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"Hello From STM32\r\n",strlen("Hello From STM32\r\n"),HAL_MAX_DELAY);
			}
			//识别到交叉路口
			else if(strcmp(&PiRxStrBuf,"detect cross")==0)//车辆停转
			{
			   cross_detected=1;
			}
			//识别到巡线信息
			else if(sscanf((const char *)&PiRxStrBuf,"cx: %d",TempCx)==1)
			{
               Angle_PID_SetTargetX(TempCx);
               Angle_PID_Update();
			}
			PiRxStrFlag=0;
		}



	if(Velocity_Plot_Indicate==1)
   {
	 Velocity_Plot();
   }



    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
    {
    	Velocity_PID_UpdateFlag=0;
    	GetVelocity();//更新左右轮转�????????
       Velocity_PID_Update();//速度PID控制
    }

	 if( TurnStopFlag==1)
	 {
		  TurnCnt=0;
		  DC_Stop();
		  TurnFlag=0;
		  //TurnStopFlag=0;
	 }

	 if(StraightStopFlag==1)
	 {
		DC_Stop();
		TotalDistance=0;
		DistanceFlag=0;
		//StraightStopFlag=0;
	 }


	//snprintf(VelocityStr,sizeof(VelocityStr),"%.2f,%.2f\r\n",v_BR,BR_Velocity_PID.TargetVelocity);//串口发�?�，绘制当前左轮速度和目标�?�度波形
	//HAL_UART_Transmit(&hlpuart1,VelocityStr,strlen(VelocityStr),HAL_MAX_DELAY);
  }

  //速度很小时关停电�?
  /*if(fabs(v_BL)<0.01 && fabs(v_BR)<0.01 && VelocityStopFlag==1)
  {
	   MoveFlag=0;
	   VelocityStopFlag=0;
	   Velocity_PID_Reset();
	   HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	   HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  }*/



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

      //速度pid调控周期�?????????????????????20ms
      if(MoveFlag==1)
      {
    	  Cnt_1ms++;
		  if(Cnt_1ms%T_velocity==0)
		  {
			  //GetVelocity();//更新左右轮转�????????
              //Velocity_Update();//速度PID控制
			  Velocity_PID_UpdateFlag=1;
			  /*占空�????????-电机转�?�关系测�????????
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

		  if(TurnFlag==1)
		  {
			  TurnCnt++;
			  if(TurnCnt>=TurnPeriod)
			      TurnStopFlag=1;
		  }
		  if(WaitFlag)
		  {
			  WaitCnt++;
		  }
      }


  }
}
//串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//串口�???????????????
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
	//和stm32通信（与树莓派间接�?�信�???????????????
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
	if(*val>0)
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
	tmp1 = tempSpeed/1000+0x30;  //千位�????????
	tmp2 = tmp0%100;
	tmp3 = tmp0/100+0x30;  //百位�????????
	tmp4 = tmp2%10;
	tmp5 = tmp2/10+0x30;  //十位�????????
	tmp6 = tmp4+0x30;  //个位�????????

	speed[0] = tmp3;
	speed[1] = tmp5;
	speed[2] = tmp6;

	HAL_UART_Transmit(&huart2,StartSTR,3,10);
	HAL_UART_Transmit(&huart2,HeaderTxBuffer5,sizeof(HeaderTxBuffer5)-1,10);
	HAL_UART_Transmit(&huart2,speed,3,10);
	HAL_UART_Transmit(&huart2,EndSTR,3,10);

}

uint8_t OpenForward()
{
	static int complete = 0; // 返回是否完成， 以构成动作链
	static int Flag = 0; // 单次执行

	if (complete) return 1;

	if (LoopStart)
	{
		complete = 0;
		Flag = 0;
		LoopStart = 0;
	}

	if (!Flag && !complete)
	{
		DC_Forward(OpenDis - 5, 0);
		Flag = 1;
	}
	if (StraightStopFlag)
	{
		complete = 1;
		Flag = 0;
		LoopStart = 1;
	}
	return complete;
}

uint8_t OpenTurn(uint8_t dir, uint8_t angle)
{
	static int complete = 0; // 返回是否完成， 以构成动作链
	static int Flag = 0; // 单次执行

	if (complete) return 1;

	if (LoopStart)
	{
		complete = 0;
		Flag = 0;
		LoopStart = 0;
	}

	if (!Flag && !complete)
	{
		DC_Turn(dir, angle);
		Flag = 1;
	}
	if (TurnStopFlag)
	{
		complete = 1;
		Flag = 0;
		LoopStart = 1;
	}
	return complete;
}


void openLoopTurning(uint8_t clockwise,uint8_t angle)
{
    uint16_t TurnCnt=0;
    uint16_t TurnPeriod=0;
	if(clockwise > 0)//右转
	{
		if(angle == 90)
		{
            DC_Start(1);
			TurnPeriod=1000;
		}
		if(angle == 180)
		{
			DC_Start(1);
			TurnPeriod=2000;
		}
	}
	else
	{
		if(angle == 90)
		{
			DC_Start(2);
			TurnPeriod=1000;
		}
		if(angle == 180)
		{
			DC_Start(2);
			TurnPeriod=2000;
		}
	}
	while(TurnCnt++<=TurnPeriod)
	{
		//速度pid更新
	    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
	    {
	    	Velocity_PID_UpdateFlag=0;
	    	GetVelocity();//更新左右轮转�????????
	       Velocity_PID_Update();//速度PID控制
	    }
	}
	DC_Stop();
}

void openLoopForward(uint8_t forwardVelocity, uint8_t forwardTime)
{
	if(MoveFlag==0)
	{
		DC_Start(0);//启动电机
	}
	uint16_t forwardCnt=0;
	uint16_t forwardPeriod=forwardTime;
	while(forwardCnt++<=forwardPeriod)
	{
		//速度pid更新
	    if(MoveFlag==1 && Velocity_PID_UpdateFlag==1)
	    {
	    	Velocity_PID_UpdateFlag=0;
	    	GetVelocity();//更新左右轮转�????????
	       Velocity_PID_Update();//速度PID控制
	    }

	    //角度pid更新
	   	if(PiRxStrFlag==1)
	   	{

            uint8_t TempCx=0;
			//识别到巡线信息
			if(sscanf((const char *)&PiRxStrBuf,"cx: %d",TempCx)==1)
			{
               Angle_PID_SetTargetX(TempCx);
               Angle_PID_Update();
			}
			PiRxStrFlag=0;
		}
	}


	DC_Stop();
	/*
	BL_SetVelocity(forwardVelocity);
	BR_SetVelocity(forwardVelocity);
	HAL_Delay(forwardTime);
	DC_Stop();
	*/
}

void Enable_CrossDetected(void)
{
	  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"enable cross detected\r\n",strlen("enable cross detected\r\n"),HAL_MAX_DELAY);//等待交叉路口检测
}

void Enable_BlockDetected(void)
{
	  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"enable block detected\r\n",strlen("enable block detected\r\n"),HAL_MAX_DELAY);//等待交叉路口检测
}

void Require_Numbers(uint8_t num)
{
  if(num==1)
	  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"detect one number\r\n",strlen("detect one number\r\n"),HAL_MAX_DELAY);//识别一个数字
  else if(num==2)
	  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"detect two numbers\r\n",strlen("detect two numbers\r\n"),HAL_MAX_DELAY);//识别两个数字
  else if(num==4)
	  HAL_UART_Transmit(&hlpuart1,(const uint8_t *)"detect four numbers\r\n",strlen("detect four numbers\r\n"),HAL_MAX_DELAY);//识别四个数字
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
