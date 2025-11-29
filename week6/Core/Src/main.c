
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>


#define ENCODER_PULSE_PER_ROT 26  // MG310正交解码后脉冲数
#define CONTROL_PERIOD 120        // 稳定控制周期
#define MAX_PWM 500               // 最大PWM（不超速）
#define MIN_PWM 30                // 最小PWM（低速稳定）

//PID参数
#define Kp 30.0f   // 极小比例系数，仅轻微微调
#define Ki 0.5f    // 几乎关闭积分，避免累积降速
#define Kd 10.0f   // 增大微分，稳定转速不波动

// 全局变量
uint8_t speed_mode = 2;          // 0=停止，1=1rps，2=2rps，3=3rps
float target_speed = 2.0f;       // 目标转速
float actual_speed = 0.0f;       // 实际转速
uint16_t pwm_duty = 200;         // 初始PWM
uint16_t pwm_lock = 200;         // PWM锁定值
int32_t encoder_cnt = 0;         // 编码器脉冲数
uint32_t last_time = 0;          // 时间戳
uint8_t speed_changed = 0;       // 转速切换标记
uint32_t speed_lock_timer = 0;   // 锁定计时器（1秒）

//PID变量 
float pid_error = 0.0f;
float pid_error_prev = 0.0f;
float pid_integral = 0.0f;
float pid_derivative = 0.0f;
float pid_output = 0.0f;

//声明
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Key_Scan_NonBlock(void);
void Encoder_Read(void);
void Speed_Calculate(void);
void PID_Control(void);
void Set_Motor_Speed(uint16_t duty);

/* USER CODE BEGIN 0 */
/**
  * @brief  按键
  */
void Key_Scan_NonBlock(void)
{
  static uint8_t key_state = 0;
  static uint32_t key_timer = 0;

  switch(key_state)
  {
    case 0:
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
      {
        key_state = 1;
        key_timer = HAL_GetTick();
      }
      break;

    case 1:
      if(HAL_GetTick() - key_timer >= 30)
      {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
          speed_mode = (speed_mode + 1) % 4;
          // 稳定PWM锁定值
          switch(speed_mode)
          {
            case 0: 
              target_speed = 0.0f; 
              pwm_lock = 0;
              speed_changed = 1;
              speed_lock_timer = HAL_GetTick();
              break;
            case 1: 
              target_speed = 1.0f; 
              pwm_lock = 80;  // 1rps稳定PWM
              speed_changed = 1;
              speed_lock_timer = HAL_GetTick();
              break;
            case 2: 
              target_speed = 2.0f; 
              pwm_lock = 200; // 2rps稳定PWM
              speed_changed = 1;
              speed_lock_timer = HAL_GetTick();
              break;
            case 3: 
              target_speed = 3.0f; 
              pwm_lock = 300; // 3rps稳定PWM
              speed_changed = 1;
              speed_lock_timer = HAL_GetTick();
              break;
          }
          pid_integral = 0.0f;
          pid_error_prev = 0.0f;
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
          key_state = 2;
        }
        else
        {
          key_state = 0;
        }
      }
      break;

    case 2:
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
      {
        key_state = 0;
      }
      break;
  }
}

/**
  * @brief  编码器读取
  */
void Encoder_Read(void)
{
  static int32_t last_cnt = 0;
  int32_t current_cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
  encoder_cnt = current_cnt - last_cnt;
  if(abs(encoder_cnt) <= 1) encoder_cnt = 0; // 过滤微小干扰
  last_cnt = current_cnt;
}

/**
  * @brief  转速计算（增加误差容忍度）
  */
void Speed_Calculate(void)
{
  if(HAL_GetTick() - last_time >= CONTROL_PERIOD)
  {
    Encoder_Read();
    actual_speed = (float)abs(encoder_cnt) / ENCODER_PULSE_PER_ROT / ((float)CONTROL_PERIOD / 1000.0f);
    last_time = HAL_GetTick();
  }
}

/**
  * @brief  PID控制
  */
void PID_Control(void)
{
  // 锁定1秒，确保转速稳定
  if(HAL_GetTick() - speed_lock_timer < 1000)
  {
    pid_output = pwm_lock;
  }
  else if(target_speed == 0.0f)
  {
    pid_output = 0;
    pid_integral = 0;
  }
  else
  {
    pid_error = target_speed - actual_speed;

    // 只有实际转速低于目标转速时，才允许积分累积（只提不降
    if(pid_error > 0.1f) // 实际转速比目标低0.1rps以上才调整
    {
      pid_integral += pid_error * ((float)CONTROL_PERIOD / 1000.0f);
      pid_integral = (pid_integral > 3.0f) ? 3.0f : pid_integral; // 积分上限极低
    }
    else
    {
      pid_integral = 0; // 否则清空积分，不降速
    }

    // 微分项稳定转速，避免波动
    pid_derivative = (pid_error - pid_error_prev) / ((float)CONTROL_PERIOD / 1000.0f);

    // PID输出：仅轻微调整，以锁定PWM为基准
    pid_output = pwm_lock + Kp * pid_error + Ki * pid_integral + Kd * pid_derivative;

    // PWM只允许在锁定值以上微调，不低于锁定值（无降速）
    if(pid_output < pwm_lock) pid_output = pwm_lock;
    // 微调范围限制在±10，无明显变化
    if(pid_output > pwm_lock + 10) pid_output = pwm_lock + 10;

    // 边界限制
    if(pid_output > MAX_PWM) pid_output = MAX_PWM;
    if(pid_output < MIN_PWM && target_speed > 0) pid_output = MIN_PWM;

    pid_error_prev = pid_error;
  }
  pwm_duty = (uint16_t)pid_output;
  Set_Motor_Speed(pwm_duty);
}

/**
  * @brief  电机控制（稳定驱动）
  */
void Set_Motor_Speed(uint16_t duty)
{
  if(duty == 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  last_time = HAL_GetTick();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  /* USER CODE END 2 */
  while (1)
  {
    Key_Scan_NonBlock();
    Speed_Calculate();
    PID_Control();
    HAL_Delay(15);
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // PA4/AIN1、PA5/AIN2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA0/按键
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA1/LED
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
	