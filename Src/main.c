/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include"ssd1306.h"
#include "fonts.h"
#include "stdlib.h"
#include "u8g_arm.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RTC_TimeTypeDef TimeKeep;

uint16_t charge_time_minutes, charge_time_hours, i;
uint16_t charge_time, charge_time_previous, discharge_time,
		discharge_time_previous, battery_low_time1, battery_low_time0;
uint8_t low_flag,cut_flag;

char seconds_s[5], minutes_s[5], hours_s[5], bat_low_s[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */













/* USER CODE BEGIN 1 */






/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
	ssd1306_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		if (HAL_GPIO_ReadPin(Charge_Status_GPIO_Port, Charge_Status_Pin)
				== GPIO_PIN_RESET) {
			HAL_RTC_GetTime(&hrtc, &TimeKeep, RTC_FORMAT_BIN);

			if (charge_time_previous != TimeKeep.Minutes) {
				charge_time++;
				charge_time_previous = TimeKeep.Minutes;
			}

		}

		if (HAL_GPIO_ReadPin(LED_Switch_GPIO_Port, LED_Switch_Pin)
				== GPIO_PIN_RESET) {
			HAL_RTC_GetTime(&hrtc, &TimeKeep, RTC_FORMAT_BIN);

			if (discharge_time_previous != TimeKeep.Minutes) {
				discharge_time++;
				discharge_time_previous = TimeKeep.Minutes;
			}
		}

		if (HAL_GPIO_ReadPin(Battery_Low_GPIO_Port, Battery_Low_Pin)
				== GPIO_PIN_RESET && low_flag == 0) {
			low_flag = 1;
			battery_low_time0 = discharge_time;
		}


		if (HAL_GPIO_ReadPin(LED_Switch_GPIO_Port, LED_Switch_Pin)
				== GPIO_PIN_SET && low_flag == 1 && cut_flag==0) {
			cut_flag=1;
			battery_low_time1=discharge_time-battery_low_time0;


		}




		            itoa(charge_time, minutes_s, 10);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Charging", Font_11x18, White);
					ssd1306_SetCursor(0, 22);
					ssd1306_WriteString("Time", Font_11x18, White);
					ssd1306_SetCursor(0, 42);
					ssd1306_WriteString(minutes_s, Font_11x18, White);
					ssd1306_UpdateScreen();
					HAL_Delay(1000);


		            itoa(discharge_time, minutes_s, 10);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Disharging", Font_11x18, White);
					ssd1306_SetCursor(0, 22);
					ssd1306_WriteString("Time", Font_11x18, White);
					ssd1306_SetCursor(0, 42);
					ssd1306_WriteString(minutes_s, Font_11x18, White);
					ssd1306_UpdateScreen();
					HAL_Delay(1000);


		            itoa(battery_low_time0, minutes_s, 10);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Full to low", Font_11x18, White);
					ssd1306_SetCursor(0, 22);
					ssd1306_WriteString("Time", Font_11x18, White);
					ssd1306_SetCursor(0, 42);
					ssd1306_WriteString(minutes_s, Font_11x18, White);
					ssd1306_UpdateScreen();
					HAL_Delay(1000);



		            itoa(battery_low_time1, minutes_s, 10);
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					ssd1306_WriteString("Low to cut", Font_11x18, White);
					ssd1306_SetCursor(0, 22);
					ssd1306_WriteString("Time", Font_11x18, White);
					ssd1306_SetCursor(0, 42);
					ssd1306_WriteString(minutes_s, Font_11x18, White);
					ssd1306_UpdateScreen();
					HAL_Delay(1000);




	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
//while(1)
	// {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_Delay(1000);
	//HAL_I2C_DeInit(&SSD1306_I2C_PORT);
	//MX_I2C1_Init();

	// }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
