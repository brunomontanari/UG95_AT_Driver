/**
******************************************************************************
* @file    main.c
* @author  Central LAB
* @version V2.0.0
* @date    22-June-2016
* @brief   Main program body. 
*          Contains C main() routine. This routine calls all other entry routines.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include "main.h"
#include "version.h"
/* USER CODE BEGIN Includes */
#include "wifi_module.h"
#include "GSM.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char pDataRX[200];
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
/* USER CODE END PFP */



int main(void)
{

    /* MCU Configuration----------------------------------------------------------*/

    /** 
    Reset of all peripherals, Initializes the Flash interface and the Systick. 
    */
    HAL_Init();

    
    /** 
    Configure the system clock 
    */
    SystemClock_Config();
    UART_GSM_Init();
    gsmInit();
    gsm_MS_Init();
    UART_Write("AT/r/n");
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,2,TimeOut_RX);
    HAL_Delay(100);
    UART_Write("ATZ"); //reset config module sim
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,2,TimeOut_RX);
    HAL_Delay(100);
    UART_Write("ATE"); //echo on
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,2,TimeOut_RX);
    HAL_Delay(100);
    UART_Write("AT+COPS?"); //check sim service provider 
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,100,TimeOut_RX);
    HAL_Delay(1000);
    UART_Write("AT+CSQ");//check signal quality
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,10,TimeOut_RX);
    HAL_Delay(1000);
    UART_Write("ATEO"); //echo off
    HAL_UART_Receive(&UartGSMHandle,(uint8_t *) pDataRX,2,TimeOut_RX);
    HAL_Delay(100);
//    gsm_GPRS_Init();
    while(1){
      gsmPoll();
      HAL_Delay(10);
    }
    /** 
    Initialize LED2. Used to report Error 
    */
    BSP_LED_Init(LED2);

    /** 
    Initialize Print Uart Port
    */
#ifdef USART_PRINT_MSG
    UART_Msg_Gpio_Init();
    USART_PRINT_MSG_Configuration(115200);
#endif  
    version_banner();

    /** 
    Initialize WIFI
    */
    
    while(wifi_main() !=WiFi_MODULE_SUCCESS)
    {
        BSP_LED_On(LED2); 
    }
    HAL_Delay(500);
    BSP_LED_Off(LED2);

    /** 
    Initialize Sensors
    */
    InitSensors();
    
    /** 
    Initialize User Button
    */
    UserButton_Init();
    
    /** 
    Initialize FreeRtos and alog with that the AWS IoT Client Thread
    */
    if(freertos_main() !=HAL_OK)
    {
        printf("freertos_main error \r\n");
    }

    
    /** 
    * Should never get here as control is now taken by the scheduler 
    */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        BSP_LED_On(LED2); ///Come here if there is Error. Switch on LED2
    }

}

void gsmEvent(char GsmEventType){
  
}
/** 
*/
/**
* @brief System Clock Configuration
* @param None
* @retval None
*/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
