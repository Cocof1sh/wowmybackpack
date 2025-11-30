#include "stm32f10x.h"
#include "Delay.h"
#include "robot.h"
#include "Key.h"

//全局变量（毫秒计时用）
volatile uint32_t sysTick_ms = 0;  // 记录系统运行毫秒数

//8路循迹模块定义
#define TRACK_CH1  ADC_Channel_7  // 1路=PA7（最左侧）
#define TRACK_CH2  ADC_Channel_6  // 2路=PA6（左外侧）
#define TRACK_CH3  ADC_Channel_5  // 3路=PA5（左内侧）
#define TRACK_CH4  ADC_Channel_4  // 4路=PA4（左中间）
#define TRACK_CH5  ADC_Channel_3  // 5路=PA3（右中间，直行！）
#define TRACK_CH6  ADC_Channel_2  // 6路=PA2（右内间，直行！）
#define TRACK_CH7  ADC_Channel_1  // 7路=PA1（右外侧）
#define TRACK_CH8  ADC_Channel_0  // 8路=PA0（最右侧）

//参数
#define THRESHOLD  2000          // 黑白线区分阈值
#define SPEED_STRAIGHT  85       // 速度
#define SPEED_TURN_SHARP  98	 // 直角弯转向速度
#define SPEED_TURN_SOFT  80      // 普通偏移修正速度
#define STOP_DELAY  100          // 停稳延时（抵消惯性）
#define TURN_90_TIME  540     // 转弯时间
#define RECTIFY_AFTER_TURN  300  // 回正时间（充分对准黑线）
#define SAME_SIDE_TRIGGER_CNT  3 // 同侧≥3个传感器=直角弯
#define TURN_LOCK_DELAY  1000     // 转向后锁死延时（避免连续拐）

//定时器2初始化（毫秒计时）
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM2时钟（APB1总线）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // TIM2配置：72MHz时钟→1MHz计数频率（1us/次），计数1000次=1ms
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;  // 自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 预分频器（72MHz/72=1MHz）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能TIM2更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 中断优先级配置（最低优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
}

//定时器2中断服务函数（1ms触发一次，累计毫秒数）
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        sysTick_ms++;  // 每1ms加1
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  // 清除中断标志
    }
}

//自定义的毫秒计时函数
uint32_t GetTick_ms(void)
{
    return sysTick_ms;
}

//ADC初始化（8路传感器读取
void ADC_InitConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    // 使能GPIOA和ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    // 配置PA0-PA7为模拟输入（传感器通道）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                               GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入，无上下拉
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC1配置：单次、单通道、独立模式
    ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    // 使能ADC1并校准
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

//传感器读取函数（稳定
uint16_t Track_Read(uint8_t ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_55Cycles5);  // 稳定采样时间
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));  // 等待采样完成
    return ADC_GetConversionValue(ADC1);
}

// 主函数
int main(void)
{
    // 初始化所有外设
    robot_Init();        // 电机初始化
    Key_Init();          // 按键初始化
    ADC_InitConfig();    // ADC传感器初始化
    TIM2_Init();         // 定时器2初始化（毫秒计时）

    while(Key_GetNum() == 0);  // 等待按键启动（防误触发）

    // 状态变量
    uint8_t is_turning = 0;                // 转向中标志（避免重复触发）
    uint32_t last_turn_time = 0;           // 上次转向完成时间
    uint8_t last_turn_side = 0;            // 上次转向侧（1=左，2=右，0=无）

    while (1)
    {
        // 读取8路传感器状态（0=黑线，1=白线）
        uint8_t ch1 = (Track_Read(TRACK_CH1) < THRESHOLD) ? 0 : 1;
        uint8_t ch2 = (Track_Read(TRACK_CH2) < THRESHOLD) ? 0 : 1;
        uint8_t ch3 = (Track_Read(TRACK_CH3) < THRESHOLD) ? 0 : 1;
        uint8_t ch4 = (Track_Read(TRACK_CH4) < THRESHOLD) ? 0 : 1;
        uint8_t ch5 = (Track_Read(TRACK_CH5) < THRESHOLD) ? 0 : 1;
        uint8_t ch6 = (Track_Read(TRACK_CH6) < THRESHOLD) ? 0 : 1;
        uint8_t ch7 = (Track_Read(TRACK_CH7) < THRESHOLD) ? 0 : 1;
        uint8_t ch8 = (Track_Read(TRACK_CH8) < THRESHOLD) ? 0 : 1;

        //计算同侧触发传感器数量
        uint8_t left_trigger_cnt = (ch1==0 ? 1:0) + (ch2==0 ? 1:0) + (ch3==0 ? 1:0) + (ch4==0 ? 1:0);
        uint8_t right_trigger_cnt = (ch5==0 ? 1:0) + (ch6==0 ? 1:0) + (ch7==0 ? 1:0) + (ch8==0 ? 1:0);

        //获取当前时间（用于判断锁死延时）
        uint32_t current_time = GetTick_ms();

        //转向中，不响应其他触发
        if (is_turning == 1)
        {
            Delay_ms(20);
            is_turning = 0;
            last_turn_time = current_time;  // 记录转向完成时间
            continue;
        }

        //转向后锁死同侧直角弯（避免连续拐）
        uint8_t can_turn_left = (left_trigger_cnt >= SAME_SIDE_TRIGGER_CNT) &&
                               ((last_turn_side != 1) || (current_time - last_turn_time > TURN_LOCK_DELAY));
        uint8_t can_turn_right = (right_trigger_cnt >= SAME_SIDE_TRIGGER_CNT) &&
                                ((last_turn_side != 2) || (current_time - last_turn_time > TURN_LOCK_DELAY));

     //动作逻辑执行
        if (ch5 == 0 && ch6 == 0)
        {
            // 直行：中间两路触发，重置转向记录
            makerobo_run(SPEED_STRAIGHT, 0);
            last_turn_side = 0;
        }
        else if (can_turn_left)
        {
        // 左直角弯：满足条件+可转向
            is_turning = 1;
            last_turn_side = 1;  // 记录左侧转向
            makerobo_brake(0);          // 紧急刹车
            Delay_ms(STOP_DELAY);       // 停稳
            makerobo_Left(SPEED_TURN_SHARP, TURN_90_TIME);  // 左转
            makerobo_run(SPEED_STRAIGHT, RECTIFY_AFTER_TURN);  // 回正
        }
        else if (can_turn_right)
        {
            // 右直角弯：满足条件+可转向
            is_turning = 1;
            last_turn_side = 2;  // 记录右侧转向
            makerobo_brake(0);           // 紧急刹车
            Delay_ms(STOP_DELAY);        // 停稳
            makerobo_Right(SPEED_TURN_SHARP, TURN_90_TIME);  // 右转
            makerobo_run(SPEED_STRAIGHT, RECTIFY_AFTER_TURN);  // 400ms
        }
        else if (ch2 == 0 || ch3 == 0 || ch4 == 0)
        {
            // 普通左偏移：轻修正
            makerobo_Left(SPEED_TURN_SOFT, 0);
        }
        else if (ch7 == 0 || ch6 == 0 || ch5 == 0)
        {
            // 普通右偏移：轻修正
            makerobo_Right(SPEED_TURN_SOFT, 0);
        }
        else if (ch1==0 && ch2==0 && ch3==0 && ch4==0 && ch5==0 && ch6==0 && ch7==0 && ch8==0)
        {
            // 异常全黑：停止
            makerobo_brake(0);
        }
        else
        {
            // 其他情况：默认直行
            makerobo_run(SPEED_STRAIGHT, 0);
        }

        Delay_ms(8);  // 主循环延时，避免传感器读取过频
    }
}
