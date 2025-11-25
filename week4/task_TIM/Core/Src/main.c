/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t freq_index = 0;  // 频率索引：0=1Hz, 1=0.5Hz, 2=0.25Hz
uint8_t led_red_state = 0;  // 红灯状态 (0=灭, 1=亮) 对应PB0引脚
uint8_t led_green_state = 0; // 绿灯状态 (0=灭, 1=亮) 对应PB1引脚
uint8_t led_blue_state = 0;  // 蓝灯状态 (0=灭, 1=亮) 对应PA5引脚

/* Private function prototypes -----------------------------------------------*/
void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // 定时器中断回调函数
void switch_timer_freq(void); // 频率切换函数
void led_toggle(GPIO_TypeDef* GPIOx, uint16_t led_pin, uint8_t *led_state); // LED翻转函数

/* Private user code ---------------------------------------------------------*/
/* Main program --------------------------------------------------------------*/
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();

  //启动TIM2定时器的中断模式（基础定时器模式）
  HAL_TIM_Base_Start_IT(&htim2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(10000);  // 延时10秒，用于触发频率切换
    switch_timer_freq();  // 执行频率切换

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//TIM周期溢出回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    switch(freq_index)
    {
      case 0:  // 1Hz频率 - 红灯(PB0)闪烁
        led_toggle(LED_RED_GPIO_Port, LED_RED_Pin, &led_red_state);
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); // 绿灯灭
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);  // 蓝灯灭
        break;
      case 1:  // 0.5Hz频率 - 绿灯(PB1)闪烁
        led_toggle(LED_GREEN_GPIO_Port, LED_GREEN_Pin, &led_green_state);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);    // 红灯灭
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);  // 蓝灯灭
        break;
      case 2:  // 0.25Hz频率 - 蓝灯(PA5)闪烁
        led_toggle(LED_BLUE_GPIO_Port, LED_BLUE_Pin, &led_blue_state);
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);    // 红灯灭
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET); // 绿灯灭
        break;
      default:
        freq_index = 0; // 索引异常时重置为默认值
        break;
    }
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

//频率切换函数：动态调整TIM2的自动重载值(ARR)
void switch_timer_freq(void)
{
  freq_index = (freq_index + 1) % 3; // 循环切换3种频率模式

  __HAL_TIM_DISABLE(&htim2); // 暂时关闭定时器，避免切换时误触发中断

  // 根据频率索引设置对应的ARR值
  switch(freq_index)
  {
    case 0:  // 1Hz：计算公式 (65535+1)*(1097+1)/72000000 ≈ 1秒（周期）
      __HAL_TIM_SET_AUTORELOAD(&htim2, 1097);
      break;
    case 1:  // 0.5Hz：计算公式 (65535+1)*(2195+1)/72000000 ≈ 2秒（周期）
      __HAL_TIM_SET_AUTORELOAD(&htim2, 2195);
      break;
    case 2:  // 0.25Hz：计算公式 (65535+1)*(4391+1)/72000000 ≈ 4秒（周期）
      __HAL_TIM_SET_AUTORELOAD(&htim2, 4391);
      break;
  }

  __HAL_TIM_ENABLE(&htim2); // 重新使能定时器，新频率立即生效
}

// LED翻转函数：适配不同GPIO端口(PA5/PB0/PB1)
void led_toggle(GPIO_TypeDef* GPIOx, uint16_t led_pin, uint8_t *led_state)
{
  if (*led_state == 0)
  {
    HAL_GPIO_WritePin(GPIOx, led_pin, GPIO_PIN_SET); // 点亮LED
    *led_state = 1;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOx, led_pin, GPIO_PIN_RESET); // 熄灭LED
    *led_state = 0;
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz ×9 =72MHz 系统时钟

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1总线最大支持36MHz，所以2分频
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM2 Initialization Function  */

/* TIM2 MspPostInit Function  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0-WKUP     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* Error Handler Function */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq(); // 关闭全局中断
  while (1)
  {
    // 死循环，方便调试时定位错误
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/* USER CODE BEGIN 4 */
void assert_failed(uint8_t *file, uint32_t line)
{
  __NOP(); // 空操作
}
/* USER CODE END 4 */
#endif
