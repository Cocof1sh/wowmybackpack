#include "main.h"
#include "tim.h"
#include "gpio.h"

void Motor_SetSpeed(uint16_t duty);
void Motor_SetDirection(uint8_t dir);

int main(void)
{
  HAL_Init();
  MX_GPIO_Init();
  MX_TIM2_Init();

  //启动TIM2CH4通道PWM输出（PA3
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // 电机初始状态：正转 + 50%占空比
  Motor_SetDirection(0);
  Motor_SetSpeed(500);

	while (1)
  {
    // 占空比从0%递增到100%（步长50，对应5%）
    for (uint16_t duty = 0; duty <= 1000; duty += 50)
    {
      Motor_SetSpeed(duty);
      HAL_Delay(200);
    }
    // 占空比从100%递减到0%
    for (uint16_t duty = 1000; duty >= 0; duty -= 50)
    {
      Motor_SetSpeed(duty);
      HAL_Delay(200);
    }
  }
}

/**
  * @brief  设置电机PWM占空比（0~1000对应0%~100%）
  * @param  duty: 占空比数值（0~1000）
  */
void Motor_SetSpeed(uint16_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
}

/**
  * @brief  设置电机方向
  * @param  dir: 0=正转（PA5=高, PA6=低）；1=反转（PA5=低, PA6=高）
  */
void Motor_SetDirection(uint8_t dir)
{
  if (dir == 0)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // AIN1（PA5）置高
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // AIN2（PA6）置低
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // AIN1（PA5）置低
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // AIN2（PA6）置高
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 配置外部晶振（HSE）
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // 配置系统时钟
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1=36MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // APB2=72MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();//错误处理用的

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif