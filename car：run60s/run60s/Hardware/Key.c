#include "stm32f10x.h"                  // Device header
#include "Delay.h"                       
//我的小车发车有一个四脚按钮

void Key_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA时钟（按键PA15）
    
    GPIO_InitTypeDef GPIO_InitStructure; // GPIO初始化结构体
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 模式：上拉输入（默认高电平，按下变低）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; // 引脚：PA15（按键连接的引脚）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速率：50MHz（没啥用，但是完整点，这是乌龟的屁股！
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 初始化GPIOA的PA15引脚
}

// 按键读取函数：检测按键是否按下，返回按键值（带消抖）
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0; // 按键值：0=未按下，1=已按下
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0) // 检测PA15是否为低电平（按键按下）
    {
        Delay_ms(20); // 延时20ms消抖（避免按键机械抖动误触发）
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0); // 等待按键松开（保持低电平时循环）
        Delay_ms(20); // 延时20ms消抖（松开时的抖动）
        KeyNum = 1; // 按键有效，设为1
    }
    return KeyNum; // 返回按键值（0=未按，1=已按）
}
