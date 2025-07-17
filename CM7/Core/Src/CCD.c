#include "CCD.h"

void Linear_CCD_Read(uint16_t* CCDADV)//一次性读取线阵CCD128像素值
{
	/*采集开始条件：SI高电平【持续20us】,CLK上升沿*/
	CCD_SI_H;//SI高电平
	DWT_Delay_us(1);//延时10us
	CCD_CLK_L;//拉低时钟
	DWT_Delay_us(10);//延时10us
	CCD_CLK_H;//时钟上升沿
	DWT_Delay_us(11);//延时11us，保证SI持续高电平时间大于20us
	CCD_SI_L;//保证SI在下一时钟上升沿之前拉低
	DWT_Delay_us(1);//延时10us

	/*128个时钟周期读取128个像素值*/
	for(uint8_t i=0;i<128;i++)
	{
       CCD_CLK_L;//时钟低电平时读取像素值
       DWT_Delay_us(1);//延时1us
	   CCDADV[i]=ADC_GetValue();//读取像素值
	   CCD_CLK_H;//拉高时钟
	   DWT_Delay_us(1);//延时1us
	}
	/*最后一个时钟周期，使pixel 128 积分电容处于采样状态*/
	CCD_CLK_L;//拉低时钟
	DWT_Delay_us(1);//延时1us
	CCD_CLK_H;//拉高时钟
	DWT_Delay_us(1);//延时1us

}


uint16_t ADC_GetValue()//ADC单通道单次转换读取值
{
	uint16_t ret;
	HAL_StatusTypeDef HalState;
	HAL_ADC_Start(&hadc1);//开启ADC转换
	HalState=HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);//等待转换完成，超时时间1ms，不停判断转换结束标志（EOC）是否置1
	if(HalState==HAL_OK)//转换成功
	{
	  ret=HAL_ADC_GetValue(&hadc1);//获取转换结果，EOC置0，等待下次转换
	}
	else{//转换异常
	  ret=0;
	}
	return ret;
}

/*CCD像素处理红线中心检测*/
void CCD_Data_Process(void)
{
   DxMax=0;
   DxMin=0;

   //中值滤波处理

    // 对每个点计算左、中、右三个值的中位数并保存到 filtered_ADV
    for (int j = 0; j < 128; j++)
    {
        int left_val, current_val, right_val;

        // 处理边界条件
        left_val = (j == 0) ? ADV[j] : ADV[j - 1];  // 左边值（j=0时取当前值）
        current_val = ADV[j];                       // 当前值
        right_val = (j == 127) ? ADV[j] : ADV[j + 1]; // 右边值（j=127时取当前值）

        // 计算三个值的中位数
        int a = left_val;
        int b = current_val;
        int c = right_val;

        // 手动计算最小值和最大值
        int min_val = a;
        if (b < min_val) min_val = b;
        if (c < min_val) min_val = c;

        int max_val = a;
        if (b > max_val) max_val = b;
        if (c > max_val) max_val = c;

        // 中位数 = 总和 - 最小值 - 最大值
        filtered_ADV[j] = a + b + c - min_val - max_val;
    }


    for (int j = 0; j < 125; j++)
    {
        dX[j] = filtered_ADV[j] - filtered_ADV[j + 3]; // 使用过滤后的数据，j+3 最大为 127（当 j=124）

        if (DxMin > dX[j])
        {
            DxMin = dX[j];
            MinIdx = j;
        }
        if (DxMax < dX[j])
        {
            DxMax = dX[j];
            MaxIdx = j;
        }
    }


    //计算红线中心位置
   if (MinIdx-MaxIdx>5)
   {
     TargetIdx = (MaxIdx+MinIdx)/2.0;
   }
}

void CCD_Read(uint16_t* CCDADV)
{
  uint8_t i=0,tslp=0,j=0;
  Linear_CCD_Flush();           // flush previously integrated frame before capturing new frame
  // wait for TSL1401 to integrate new frame, exposure time control by delay
    for(j=0;j<20;j++)
    {
       Dly_us();
    }

  //TSL_SI=1;
  CCD_SI_H;
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  Dly_us();
  //TSL_CLK=1;
  CCD_CLK_H;
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  Dly_us();
  //TSL_SI=0;
  CCD_SI_L;
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  Dly_us();
  //TSL_CLK=0;
  CCD_CLK_L;
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  Dly_us();

  for(i=0;i<128;i++)					//128 DATA/LINE
  {
      /* ADC conversion completed */
      /*##-5- Get the converted value of regular channel  ########################*/
      //uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
      //Dly_us();

     CCDADV[i]=HAL_ADC_GetValue(&hadc1);//获取转换结果，EOC置0，等待下次转换



    //ADV[tslp]=(((aADCDualConvertedValues[0])&(0xFFF))+((aADCDualConvertedValues[1])&(0xFFF))+((aADCDualConvertedValues[2])&(0xFFF))+((aADCDualConvertedValues[3])&(0xFFF))+((aADCDualConvertedValues[4])&(0xFFF)))/5;

    //TSL_CLK=1;
    CCD_CLK_H;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    Dly_us();
    CCD_CLK_L;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    Dly_us();
    Dly_us();
  }
    CCD_CLK_H;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);    //129th pulse to terminate output of 128th pixel
    Dly_us();
    CCD_CLK_L;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
}

// simply generate SI & CLK pulses to flush the previously integrated frame
// while integrating new frame to be read out
void Linear_CCD_Flush(void)
{
    uint8_t index=0;
    //TSL_SI=1;
    CCD_SI_H;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    Dly_us();
    //TSL_CLK=1;
    CCD_CLK_H;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);  	// 1st Pulse
    Dly_us();
    //TSL_SI=0;
    CCD_SI_L;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    Dly_us();
    //TSL_CLK=0;
    CCD_CLK_L;
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    Dly_us();

    for(index=0; index<128; index++)
    {
          //TSL_CLK=1;
    	CCD_CLK_H;
       // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        Dly_us();
        Dly_us();
            //TSL_CLK=0;
        CCD_CLK_L;
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        Dly_us();
        Dly_us();
    }
}

void Dly_us(void)
{
  uint8_t j=0;
  for(j=0;j<64;j++)
    {Dly();}
}

void Dly(void)
{
   uint32_t ii;
   for(ii=0;ii<100;ii++);
}


