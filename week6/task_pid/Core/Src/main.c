#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal_gpio.h"

uint8_t gear = 1;
float target_rps[] = {1.0f, 1.5f, 2.0f};
float actual_rps = 0.0f;
int32_t prev_encoder_cnt = 0;
uint16_t pwm_out = 0;

// PID参数 
#define Kp 60.0f
#define Ki 10.0f
#define Kd 3.0f
float pid_err = 0.0f;
float pid_err_prev = 0.0f;
float pid_integral = 0.0f;

//编码器参数（MG310固定）
#define ENCODER_PULSE 11
#define ENCODER_MUL 4

void Get_Actual_RPS(void);
void PID_Calculate(void);
void LED_Indicator(void);

int main(void)
{
  HAL_Init();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  //启动外设 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);

  //正转配置
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

  while (1)
  {
    // 100ms更新一次
    Get_Actual_RPS();
    PID_Calculate();
    LED_Indicator();

    // 串口打印 
    char buf[80];
    sprintf(buf, "Gear:%d, Target:%.1frps, Actual:%.1frps, PWM:%d\r\n",
            gear, target_rps[gear-1], actual_rps, pwm_out);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);

    HAL_Delay(100);
  }
}

// 读取转速
void Get_Actual_RPS(void)
{
  int32_t current_cnt = __HAL_TIM_GET_COUNTER(&htim3);
  int32_t diff = current_cnt - prev_encoder_cnt;
  prev_encoder_cnt = current_cnt;
  actual_rps = (float)diff / (ENCODER_PULSE * ENCODER_MUL) / 0.1f;
}

// PID计算
void PID_Calculate(void)
{
  pid_err = target_rps[gear-1] - actual_rps;
  pid_integral += pid_err;

  // 积分限幅 
  if (pid_integral > 30) pid_integral = 30;
  if (pid_integral < -30) pid_integral = -30;

  // PID输出 
  float pid_out = Kp * pid_err + Ki * pid_integral + Kd * (pid_err - pid_err_prev);
  pwm_out = (uint16_t)pid_out;

  // PWM限幅
  if (pwm_out > 1000) pwm_out = 1000;
  if (pwm_out < 0) pwm_out = 0;

  // 更新PWM 
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_out);
  pid_err_prev = pid_err;
}

// LED指示状态
void LED_Indicator(void)
{
  switch(gear)
  {
    case 1:
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      break;
    case 2:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      HAL_Delay(350);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      break;
  }
}

// 按键中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY1_Pin)
  {
    HAL_Delay(20);
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      gear++;
      if (gear > 3) gear = 1;
      pid_integral = 0.0f;
      pid_err_prev = 0.0f;
    }
   // 清除PA0对应的外部中断0挂起标志位
EXTI->PR |= EXTI_PR_PR0;
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // KEY1配置 
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  // LED配置
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  // IN1/IN2配置 
  GPIO_InitStruct.Pin = IN1_Pin | IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 初始状态 
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

  // 中断优先级
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  TIM_Encoder_InitTypeDef sEncoderConfig = {0};
  sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sEncoderConfig.IC1Filter = 0;
  sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sEncoderConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim3, &sEncoderConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(100);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}
#endif
