/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 频率模式对应参数定义
#define FREQ_MODE_0_5HZ    0    // 0.5Hz = 2秒周期
#define FREQ_MODE_0_333HZ  1    // 0.333Hz = 3秒周期
#define FREQ_MODE_0_25HZ   2    // 0.25Hz = 4秒周期
#define LED_ON_DURATION    200  // LED点亮持续时间（ms）
#define KEY_DEBOUNCE_TIME  10   // 按键消抖时间（ms）
// LED引脚定义（PA2/PA3/PA4，与接线对应）
#define LED_RED_PIN    GPIO_PIN_2
#define LED_GREEN_PIN  GPIO_PIN_3
#define LED_BLUE_PIN   GPIO_PIN_4
#define LED_PORT       GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t freq_mode = FREQ_MODE_0_5HZ;  // 当前频率模式（默认0.5Hz）
uint8_t led_flag = 0;                 // LED点亮标记（1=需要熄灭，0=未点亮）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Key_Scan(void);  // 按键扫描函数声明
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  定时器中断回调函数（TIM2定时时间到触发）
  * @param  htim: 定时器句柄指针
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)  // 确认是TIM2触发的中断
  {
    // 根据当前频率模式点亮对应LED（PA2=红，PA3=绿，PA4=蓝）
    switch(freq_mode)
    {
      case FREQ_MODE_0_5HZ:
        HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);    // 0.5Hz → 红灯亮（PA2）
        break;
      case FREQ_MODE_0_333HZ:
        HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);  // 0.333Hz → 绿灯亮（PA3）
        break;
      case FREQ_MODE_0_25HZ:
        HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_SET);   // 0.25Hz → 蓝灯亮（PA4）
        break;
      default:
        break;
    }
    led_flag = 1;  // 标记LED已点亮，等待主循环熄灭
  }
}

/**
  * @brief  按键扫描函数（带消抖，切换频率模式）
  * @param  None
  * @retval None
  */
void Key_Scan(void)
{
  static uint8_t key_state = 0;    // 按键状态：0=未按下，1=已按下
  static uint32_t key_timer = 0;   // 消抖计时变量

  // 检测按键是否按下（PB0为低电平，内部上拉）
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
  {
    key_timer++;  // 按下状态计时
    if(key_timer >= KEY_DEBOUNCE_TIME)  // 满足消抖时间
    {
      key_state = 1;  // 标记按键已稳定按下
    }
  }
  else  // 按键松开
  {
    if(key_state == 1)  // 之前是稳定按下状态，现在松开 → 确认一次有效按键
    {
      // 切换频率模式（循环：0→1→2→0）
      freq_mode = (freq_mode + 1) % 3;

      // 根据新模式更新TIM2的自动重装值（实现不同定时周期）
      switch(freq_mode)
      {
        case FREQ_MODE_0_5HZ:  // 2秒周期 = 10kHz时钟 × 20000计数（预分频后为10kHz）
          __HAL_TIM_SET_AUTORELOAD(&htim2, 19999);
          break;
        case FREQ_MODE_0_333HZ:  // 3秒周期 = 10kHz × 30000计数
          __HAL_TIM_SET_AUTORELOAD(&htim2, 29999);
          break;
        case FREQ_MODE_0_25HZ:  // 4秒周期 = 10kHz × 40000计数
          __HAL_TIM_SET_AUTORELOAD(&htim2, 39999);
          break;
        default:
          break;
      }
      __HAL_TIM_SET_COUNTER(&htim2, 0);  // 重置计数器，新周期立即生效
      key_state = 0;  // 重置按键状态
    }
    key_timer = 0;  // 松开时重置计时
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);  // 启动TIM2定时中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key_Scan();  // 实时扫描按键，检测频率切换指令

    // 若LED已点亮，延时后熄灭（满足"触发后消除"要求）
    if(led_flag == 1)
    {
      HAL_Delay(LED_ON_DURATION);  // LED保持点亮200ms（肉眼清晰可见）
      // 熄灭所有LED（PA2/PA3/PA4），避免多灯同时亮
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);
      led_flag = 0;  // 重置LED状态标记
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
