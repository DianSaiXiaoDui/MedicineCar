#include "main.h"

#include "CCD.h"


/*
#define CCD_SI_H HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_SET);
#define CCD_SI_L HAL_GPIO_WritePin(CCD_SI_GPIO_Port, CCD_SI_Pin, GPIO_PIN_RESET);
#define CCD_CLK_H HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_SET);
#define CCD_CLK_L HAL_GPIO_WritePin(CCD_CLK_GPIO_Port, CCD_CLK_Pin, GPIO_PIN_RESET);

//微秒级延时函数
void Dly_us(void)
{
  uint8_t j=0;
  for(j=0;j<64;j++)
    {Dly();}
}

//延时
void Dly(void)
{
   uint32_t ii;
   for(ii=0;ii<100;ii++);
}


//读取CCD数据
void CCD_Read(void)
{
  uint8_t i=0,tslp=0,j=0;
  Linear_CCD_Flush();           // flush previously integrated frame before capturing new frame
  // wait for TSL1401 to integrate new frame, exposure time control by delay
  for(j=0;j<10;j++)
  {
    Dly_us();
  }

  //TSL_SI=1;
  CCD_SI_H;
  Dly_us();
  //TSL_CLK=1;
  CCD_CLK_H;
  Dly_us();
  //TSL_SI=0;
  CCD_SI_L;
  Dly_us();
  //TSL_CLK=0;
  CCD_CLK_L;
  Dly_us();

  for(i=0;i<128;i++)					//128 DATA/LINE
  {
      /* ADC conversion completed */
      /*##-5- Get the converted value of regular channel  ########################*/
      //uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
      //Dly_us();
   // ADV[tslp]= (HAL_ADC_GetValue(&hadc1))&(0xffff);
    //  ++tslp;
    //ADV[tslp]=(((aADCDualConvertedValues[0])&(0xFFF))+((aADCDualConvertedValues[1])&(0xFFF))+((aADCDualConvertedValues[2])&(0xFFF))+((aADCDualConvertedValues[3])&(0xFFF))+((aADCDualConvertedValues[4])&(0xFFF)))/5;

    //TSL_CLK=1;
    //CCD_CLK_H;
    //Dly_us();
    //TSL_CLK=0;
    //CCD_CLK_L;
    //Dly_us();
    //Dly_us();
  //}
    //CCD_CLK_H;    //129th pulse to terminate output of 128th pixel
    //Dly_us();
    //CCD_CLK_L;
//}

// simply generate SI & CLK pulses to flush the previously integrated frame
// while integrating new frame to be read out
/*void Linear_CCD_Flush(void)
{
    uint8_t index=0;
    //TSL_SI=1;
    CCD_SI_H;
    Dly_us();
    //TSL_CLK=1;
    CCD_CLK_H;
    Dly_us();
    //TSL_SI=0;
    CCD_SI_L;
    Dly_us();
    //TSL_CLK=0;
    CCD_CLK_L;
    Dly_us();

    for(index=0; index<128; index++)
    {
        //TSL_CLK=1;
    	CCD_CLK_H;
        Dly_us();
        Dly_us();
        //TSL_CLK=0;
        CCD_CLK_L;
        Dly_us();
        Dly_us();
    }
}*/

//处理CCD数据
/*void CCD_DataProcess()
{
   uint8_t j;
   DxMax=0;
   DxMin=0;

   // --- 新增中值滤波处理 ---
   if(median_filter==1)
   {
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


    for (j = 0; j < 125; j++)
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
   }
   // --- 中值滤波处理结束 ---
   else{
        for(j=0; j<125; j++)
      {
          dX[j] = ADV[j] - ADV[j+3];
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

   }

   if (MaxIdx < MinIdx&&MinIdx-MaxIdx>5)
     TargetIdx = (MaxIdx+MinIdx)>>1;

}*/
