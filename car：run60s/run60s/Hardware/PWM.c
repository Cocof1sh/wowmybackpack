#include "stm32f10x.h"                  // Device header

// PWM初始化，配置TIM4 4个通道（PB6~PB9），用于电机速度控制
void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;        // GPIO初始化结构体
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; // 定时器初始化结构体
    TIM_OCInitTypeDef TIM_OCInitStructure;      // 定时器输出比较（PWM）初始化结构体

    // 使能TIM4时钟（TIM4属于APB1总线外设）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // 使能GPIOB时钟（PWM输出引脚接PB6~PB9）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
                                                                          
    // 配置PB6为TIM4_CH1（PWM通道1）引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   // 引脚：PB6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 模式：复用推挽输出（PWM专用）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速率：50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);      // 初始化GPIOB

    // 配置PB7为TIM4_CH2（PWM通道2）引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;   // 引脚：PB7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 模式：复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速率：50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);      // 初始化GPIOB

    // 配置PB8为TIM4_CH3（PWM通道3）引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;   // 引脚：PB8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 模式：复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速率：50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);      // 初始化GPIOB

    // 配置PB9为TIM4_CH4（PWM通道4）引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   // 引脚：PB9
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 模式：复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 速率：50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);      // 初始化GPIOB

    // 配置TIM4时基参数（决定PWM频率）
    TIM_TimeBaseStructure.TIM_Period = 100 - 1; // 自动重装值：99（PWM周期=100个计数）
    TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1; // 预分频器：35（72MHz/36=2MHz计数时钟）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; // 时钟分割：无
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数模式：向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // 初始化TIM4时基

    // 配置TIM4_CH1为PWM1模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1（计数<脉冲数时输出高）
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始脉冲数：0（初始占空比0，电机停）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性：高电平有效
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // 初始化TIM4_CH1

    // 配置TIM4_CH2为PWM1模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始脉冲数：0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // 初始化TIM4_CH2

    // 配置TIM4_CH3为PWM1模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始脉冲数：0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // 初始化TIM4_CH3

    // 配置TIM4_CH4为PWM1模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 使能输出
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始脉冲数：0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 高电平有效
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // 初始化TIM4_CH4

    // 使能4个通道的输出比较预装载（PWM值更新更稳定）
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能CH1预装载
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能CH2预装载
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能CH3预装载
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能CH4预装载
    TIM_ARRPreloadConfig(TIM4, ENABLE); // 使能TIM4自动重装预装载

    TIM_Cmd(TIM4, ENABLE); // 使能TIM4，开始输出PWM
}
