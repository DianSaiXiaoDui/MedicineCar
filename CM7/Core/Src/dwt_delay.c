#include "dwt_delay.h"

/**
  * @brief 初始化 DWT 周期计数器
  * @retval 0: 失败; 1: 成功
  */
uint8_t DWT_Init(void) {
    /* 检查是否支持 DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* 重置并启用 CYCCNT */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 验证 DWT 是否可用 */
    if (DWT->CYCCNT) {
        return 1;  // 初始化成功
    } else {
        return 0;  // 芯片不支持 DWT
    }
}


/**
  * @brief 微秒级延时（基于 DWT）
  * @param us: 延时的微秒数（1~1000000）
  */
void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;

    uint32_t cycles = (us) * (SystemCoreClock / 1000000) ;  // 计算周期数

    /* 防止计数器溢出（24小时后溢出 @72MHz） */
    while ((DWT->CYCCNT - start) < cycles) {
        // 空循环等待
    }
}

/*自校准测试*/
uint32_t measure_actual_us(uint32_t us) {
	uint32_t CYCLES_PER_US= SystemCoreClock / 1000000;
    uint32_t start = DWT->CYCCNT;
    DWT_Delay_us(us);
    return (DWT->CYCCNT - start) / CYCLES_PER_US;  // 返回实际延时
}
