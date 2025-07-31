/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "nano.h"
#include "speaker.h"
#include "TJC.h"
#include "delay.h"


extern  uint8_t _uart2_byte_buf;  
extern  uint8_t _uart1_byte_buf;  
extern  uint8_t get_all_point;  
extern uint8_t Point_cnt;
extern vector2f FB_Point[];
extern GroundStation_To_TIVA_Target To_TIVA_Target; 


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */



	delay_init(168);
	
	//UART2  (NANO)  ->蓝牙 HC05    :用于主控之间的无线通
	UART_Start_Receive_IT(&huart2,&_uart2_byte_buf,1);  //开启接收中断	    


	//UART1  (舵机)  -> TJC 串口屏   :用于显示航线 观察地面小动物的信息  和人工设置禁飞区的触摸按键接口
	UART_Start_Receive_IT(&huart1,&_uart1_byte_buf,1);  //开启接收中断	 
	

		HAL_Delay(2000);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
			if(	1	/*get_all_point==1*/)
			{
					To_TIVA_Target.Fobidden_Rigion_Num=3;
					To_TIVA_Target.Fobidden_Rigion[0].A=0x01;
					To_TIVA_Target.Fobidden_Rigion[0].B=0x02;
					To_TIVA_Target.Fobidden_Rigion[1].A=0x03;
					To_TIVA_Target.Fobidden_Rigion[1].B=0x04;
					To_TIVA_Target.Fobidden_Rigion[2].A=0x05;
					To_TIVA_Target.Fobidden_Rigion[2].B=0x06;
				
				
					
					get_all_point=0;
//					for(int i=0;i<Point_cnt;i++)
//					{
//							To_TIVA_Target.Fobidden_Rigion[i].A=FB_Point[i].A;
//							To_TIVA_Target.Fobidden_Rigion[i].B=FB_Point[i].B;
//					}
	//				To_TIVA_Target.Fobidden_Rigion_Num=Point_cnt;
					GroundStation_Send_To_TIVA(&To_TIVA_Target);
					Point_cnt=0;
			
			}
//		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		 HAL_Delay(5000);
//	  
	  
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


int fputc(int ch, FILE *f)
{
    HAL_StatusTypeDef status;

    // 检查 UART 状态，确保串口处于空闲状态
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY || 
        HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX) 
    {
        status = HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

        // 检测传输是否成功
        if (status == HAL_OK) 
        {
            return ch; // 传输成功，返回字符
        }
        else 
        {
            // 传输失败处理，例如记录错误
            return EOF; // 返回错误状态
        }
    }
    else 
    {
        // 串口未准备好，返回错误状态
        return EOF;
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
