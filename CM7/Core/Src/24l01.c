#include "24l01.h"

/*
 *
 * #define SPI2_CN_Pin GPIO_PIN_6
#define SPI2_CN_GPIO_Port GPIOF
#define SPI2_IRQ_Pin GPIO_PIN_8
#define SPI2_IRQ_GPIO_Port GPIOF
#define CCD_CLK_Pin GPIO_PIN_3
#define CCD_CLK_GPIO_Port GPIOC
#define CCD_SI_Pin GPIO_PIN_5
#define CCD_SI_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
 * */
//检测24L01是否存在
//返回值:0，成功;1，失败

// 添加全局变量
volatile uint8_t nrf_rx_flag = 0;
char nrf_rx_buffer[RX_BUFFER_SIZE];
NRF_RxCallback rx_callback = NULL;
uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01};//接收地址
uint8_t aTxBuffer[32];
uint8_t aRxBuffer[32];

// 添加缺失的函数实现
void Set_RxCallback(NRF_RxCallback callback) {
    rx_callback = callback;
}

uint8_t NRF_GetRxFlag(void) {
    return nrf_rx_flag;
}

void NRF_ClearRxFlag(void) {
    nrf_rx_flag = 0;
}

// 中断服务函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI2_IRQ_Pin) {
        NRF24L01_IRQHandler();
    }
}

//修改中断处理函数
void NRF24L01_IRQHandler(void)
{
    uint8_t status = NRF24L01_Read_Reg(STATUS);
    uint8_t clear_mask = 0;

    // 处理接收中断 (使用 RX_DR 位)
    if (status & (1 << RX_DR)) {  // 使用 RX_DR 宏
        NRF24L01_Read_Buf(RD_RX_PLOAD, (uint8_t*)nrf_rx_buffer, RX_PLOAD_WIDTH);
        nrf_rx_buffer[RX_PLOAD_WIDTH] = '\0'; // 添加字符串结束符
        clear_mask |= (1 << RX_DR);  // 使用 RX_DR 宏

        // 处理接收数据
        if (rx_callback) {
            rx_callback(nrf_rx_buffer);
        } else {
            nrf_rx_flag = 1;
        }
    }

    // 清除所有中断标志
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS, status | clear_mask);
    __HAL_GPIO_EXTI_CLEAR_IT(SPI2_IRQ_Pin);
}

void NRF24L01_Init(void)//初始化时配置为接收模式
{
	NRF24L01_RX_Mode(CHANNEL);  // 进入接收模式
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);  // 使能中断线
}

uint8_t NRF24L01_Check(void)
{
    uint8_t Check_buf[5]={0xA5,0xA5,0xA5,0xA5,0xA5};
    uint8_t Check_buf1[5]={0x0,0x0,0x0,0x0,0x0};
    uint8_t i;

    //SPI1_SetSpeed(SPI_BaudRatePrescaler_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）

    __HAL_SPI_DISABLE( &hspi2);
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BAUDRATEPRESCALER_16 ));
    SPI1->CR1&=0XFFC7;
    SPI1->CR1|=SPI_BAUDRATEPRESCALER_16 ;	//  速度<9Mhz
    __HAL_SPI_ENABLE( &hspi2);

    NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,Check_buf,5);//向发送地址寄存器写入5个字节的地址,NRF_WRITE_REG(写配置寄存器，001xxxxx)+TX_ADDR(发送地址寄存器)

    NRF24L01_Read_Buf(TX_ADDR,Check_buf1,5); //将刚刚写入的数据读取出来

    for(i=0;i<5;i++)//检验发送和读取数据是否相等
    {
      if(Check_buf1[i]!=Check_buf[i])//检测到不等字节，检测24L01失败
       return 1;
    }
    //printf("%d\r\n",i);//240112
    //if(i!=5)return 1;
    return 0;		 //检测到24L01
}


//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
    uint8_t status;
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);  //使能SPI传输

     aTxBuffer[0] = reg;//待写入的寄存器地址
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //发送寄存器地址
    status = aRxBuffer[0];

    aTxBuffer[0] = value;//待写入值放入数据缓冲区
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //向指定寄存器发送数据

    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);    //禁止SPI传输
    return(status);      //返回状态值
}


//读取SPI寄存器值
//reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);  //使能SPI传输

    aTxBuffer[0] = reg;//待读取的寄存器地址
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //发送寄存器地址

    aTxBuffer[0] = 0xFF;//主机随机发送数据
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //读取指定寄存器值到缓冲器
    reg_val = aRxBuffer[0];//获取寄存器值

    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);         //禁止SPI传输
    return(reg_val);           //返回寄存器值
}


//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
    uint8_t status,uint8_t_ctr;
    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);  //使能SPI传输

    aTxBuffer[0] = reg;//寄存器地址放入输出缓冲区
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //发送寄存器地址
    status = aRxBuffer[0];//获取状态值

    for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
      {
          aTxBuffer[0] = 0xFF;//主机随机发送数据
          HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //从指定寄存器逐个读取数据
          pBuf[uint8_t_ctr] = aRxBuffer[0];   //将读取的字节放入接收数据缓冲区
      }

    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);         //禁止SPI传输
    return status;        //返回状态值
}


//在指定寄存器位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,uint8_t_ctr;

    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);  //使能SPI传输

    aTxBuffer[0] = reg;//将寄存器号放入发送数据缓冲区
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //发送寄存器号
    status = aRxBuffer[0];//获取HAL_Status

    for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
    {
      aTxBuffer[0] = pBuf[uint8_t_ctr];//逐个把待发送数据放入数据缓冲区
      HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //逐个发送数据
    }

    HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);        //关闭SPI传输

    return status;          //返回HAL_Status
}


//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{

	// 临时禁用接收中断
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    uint8_t sta;

    //SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）
    __HAL_SPI_DISABLE( &hspi2);
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BAUDRATEPRESCALER_16 ));
    SPI1->CR1&=0XFFC7;
    SPI1->CR1|=SPI_BAUDRATEPRESCALER_16 ;
    __HAL_SPI_ENABLE( &hspi2);

    /*在 发送模式 下，NRF24L01 不会立即发送数据，而是需要 CE 先拉低再拉高来触发发送。*/
    HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_RESET);  //CE=0
    NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX FIFO ，32个字节
    HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_SET); //CE=1，CE从低电平变为高电平，数据被发送出去


    while(HAL_GPIO_ReadPin(SPI2_IRQ_GPIO_Port,SPI2_IRQ_Pin)!= 0x0);  //等到IRQ=0，表示接受到应答信号
    sta = NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //在状态寄存器里面写入相同状态值，清除中断状态

    if(sta&MAX_TX)//达到最大重发次数
    {
      NRF24L01_Write_Reg(FLUSH_TX,0xff);//发送失败，清除TX FIFO寄存器
      return MAX_TX;
    }
    if(sta&TX_OK)//发送完成
    {
      return TX_OK;
    }

    // 重新使能中断并返回接收模式
    NRF24L01_RX_Mode(CHANNEL);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    return 0xff;//其他原因发送失败

}



//启动NRF24L01接收一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
        __HAL_SPI_DISABLE( &hspi2);
        assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BAUDRATEPRESCALER_16 ));
        SPI1->CR1&=0XFFC7;
        SPI1->CR1|=SPI_BAUDRATEPRESCALER_16 ;	//
        __HAL_SPI_ENABLE( &hspi2);


	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TD_Rx(接收数据中断)
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
		return 0;
	}
	return 1;//没收到任何数据
}



//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
void NRF24L01_RX_Mode(uint8_t CH)
{
        HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_RESET);  //CE=0

  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写入接收通道0寄存器地址【5字节】

  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,CH);	     //设置通信频率
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	 （一次接受的数据宽度：32字节）
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式

        HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_SET); //CE=1，进入接收模式
}



//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入TX模式,并可以接收数据了
//CE为高大于10us,则启动发送.
/*
EN_AA      Bit5~0:通道5-0自动应答使能
EN_RXADDR  Bit5~0:接收通道5~0使能
SETUP_PETR  Bit7~4:等待时间，Bit3~0：最大重发次数
RF_CH     Bit6~0:通信频率为2400+（0~124）MHz
RF_SETUP  bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
CONFIG   bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;   bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能【中断使能低电平有效】
*/

void NRF24L01_TX_Mode(uint8_t CH)
{
  HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_RESET);  //CE=0，待机模式，不会发送/接收数据

  /*进行寄存器配置*/
  NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//向发送地址寄存器写入发送地址【5字节】
  NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //向接收地址通道0寄存器写入同发送地址的接收地址（用于接受应答信号）【5字节】

  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
  NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0接收
  NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,CH);       //设置通信频率
  NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
  NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断

  HAL_GPIO_WritePin(SPI2_CN_GPIO_Port,  SPI2_CN_Pin, GPIO_PIN_SET); //CE=1，进入接收模式
}



uint8_t NRF24L01_SendString(char *tx_buf) {
    uint8_t retries = 0;
    uint8_t status;

    // 切换到发送模式
    NRF24L01_TX_Mode(CHANNEL);

    do {
        status = NRF24L01_TxPacket((uint8_t*)tx_buf);

        if(status == TX_OK) {
            return TX_OK; // 发送成功
        }

        // 发送失败处理
        retries++;
        if(retries >= MAX_TX_RETRIES) {
            break; // 达到最大重试次数
        }

        // 重试前延时5ms（避免连续发送）
        HAL_Delay(5);
    } while(1);

    return MAX_TX; // 返回重试失败
}


/*
 * 主程序检查硬件
 *
 * if (NRF24L01_Check() == 0) {
        printf("NRF24L01 Detected!\r\n");
    } else {
        printf("NRF24L01 Not Found!\r\n");
        while (1); // 死循环阻塞
    }
 *
 * */

/*发送主程序示例
 *
 *#define CHANNEL 10//无线通信频道
 *
 *发送数据缓冲区
 *char tx_buf[32] = "Hello World!"; // 发送缓冲区（最大32字节）
 *
 *
 *发送信息
 *NRF24L01_Send(tx_buf);

*/

/*接收主程序示例
 *
 *#define CHANNEL 10//无线通信频道
 *
 *发送数据缓冲区
 *char tx_buf[32] = "Hello World!"; // 发送缓冲区（最大32字节）
 *
 *
 *发送信息
 *NRF24L01_Send(tx_buf);

*/

