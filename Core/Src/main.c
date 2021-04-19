/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//手飞模式控制投放框状态机
typedef enum
{
	no_cmd = 0,
	first_block,
	first_block_done,
	second_block,
	second_block_done,
	third_block,
	third_block_done,
}Free_block;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEVO_CLOSE        TIM1->CCR1 = 1500			//舵机停转
#define SEVO_OPEN_LEFT	  TIM1->CCR1 = 1000   		//舵机左转
#define SEVO_OPEN_RIGHT	  TIM1->CCR1 = 2000			//舵机右转
#define ROTATE_TIME		  10						//时间单位是100us,舵机旋转的时间是 100*ROTATE_TIME(us)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd[4]={0};
uint8_t block_state = no_cmd;
uint8_t flag[3]={0};
uint8_t trig_flag = 0;
int32_t rotate_time = 0;
//设置舵机当前的控制指令
void set_block_control()
{
	

	//不打开投放框
	 if(block_state == no_cmd)
	{
		SEVO_CLOSE;				//舵机停转状态		
	}
	//打开第一个投放框
	else if(block_state == first_block)
	{
		if( rotate_time >0)			//更改rotate_time的转载时间改变舵机旋转的角度
			SEVO_OPEN_RIGHT;		//在投放1状态下，在固定的时间内旋转舵机
		else
		{
			block_state = first_block_done;
		}	
	}
	//第一个投放框投放结束
	else if(block_state == first_block_done)
	{
		SEVO_CLOSE;
			
	}
	//打开第二个投放框
	else if(block_state == second_block)
	{
		if( rotate_time >0)			//更改rotate_time的转载时间改变舵机旋转的角度
			SEVO_OPEN_RIGHT;		//在投放1状态下，在固定的时间内旋转舵机
		else
		{
			block_state = second_block_done;
		}	
	}
	//第二个投放框投放结束
	else if(block_state == second_block_done)
	{
		SEVO_CLOSE;
	}
	//打开第三个投放框
	else if(block_state == third_block)
	{
		if( rotate_time >0)			//更改rotate_time的转载时间改变舵机旋转的角度
			SEVO_OPEN_RIGHT;		//在投放1状态下，在固定的时间内旋转舵机
		else
		{
			block_state = third_block_done;
		}	
	}
	else if(block_state == third_block_done)
	{
		SEVO_CLOSE;
	}
	
	
	
	
	
	
	/*if ( (block_state == first_block) && (rotate_time <= 0) )
		block_state = first_block_done;
	else if( (block_state == second_block) && (rotate_time <= 0) )
		block_state = second_block_done;
	else if( (block_state == third_block) && (rotate_time <= 0) )
		block_state = third_block_done;*/
		

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);			//使能打开PWM输出通道
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

HAL_TIM_Base_Start(&htim1);
HAL_TIM_Base_Start(&htim2);
HAL_Delay(800);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	set_block_control();		 //设置投放物投放指令
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int i;
    for(i=0;i<1000;i++);	//延时一段时间消抖，不能用HAL_DELAY,防止程序卡死在该回调函数中
	while(!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6));		//如果电平没有发生改变，则阻塞在当前进程，状态不进行更新，直到完整的控制信号发出才更新状态
	
	
	
	if(block_state == third_block_done)
	{
		block_state = no_cmd;		//由投放框状态进入无命令状态时，不触发模拟边沿跳变
		flag[0] = 0;
		flag[1] = 0;
		flag[2] = 0;
	
	}
	else if ((block_state == second_block_done) && (flag[2] == 0) && trig_flag == 0)
	{
		block_state = third_block;
		trig_flag = 1;				//投放框状态的切换过程中会触发模拟边沿跳变
		flag[2] = 1;
	
	}
	else if ((block_state == first_block_done) && (flag[1] == 0) && trig_flag == 0)
	{
		block_state = second_block;
		trig_flag = 1;				//投放框状态的切换过程中会触发模拟边沿跳变
		flag[1] = 1;
	
	}
	else if ((block_state == no_cmd) && (flag[0] == 0) && trig_flag == 0)
	{
		block_state = first_block;
		trig_flag = 1;				//投放框状态的切换过程中会触发模拟边沿跳变
		flag[0] = 1;				//更新当前投放标志，不再进入该判断，防止重复刷新标志位
	}
		

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)		//当定时器2触发中断之后，进入中断回调函数，更新当前的旋转时间刻度
	{
		if(trig_flag == 1)	//在外部中断中更新中断出发标志位，模拟边沿触发
		{
			trig_flag = 0;  //表征边沿触发结束，防止反复重装载时间
			rotate_time = ROTATE_TIME;		//重装载模拟定时器时间，刻度为10
		}
		rotate_time--;		//每100us进行一次减法，表征已经过了100us
		
		if(rotate_time <= -100)
			rotate_time = -10;		//将旋转时间牵制在低位数字，防止变量溢出
		
	}





}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
