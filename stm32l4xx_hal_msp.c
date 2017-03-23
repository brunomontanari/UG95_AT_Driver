/**
  ******************************************************************************
  * File Name          : stm32l4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
#include "stm32l4xx_hal.h"
#include "main.h"
#include "wifi_module.h"
#include "wifi_globals.h"
#include "GSM.h"
    
extern void Error_Handler(void);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE BEGIN MspInit 1 */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_GSM_Pwr_Key_CLK_ENABLE();
  
  GPIO_InitStruct.Pin       = GSM_Pwr_Key_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  
  HAL_GPIO_Init(GSM_Pwr_Key_Port, &GPIO_InitStruct);
  
#ifdef gsm_reset_en
  GPIO_GSM_Reset_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GSM_Reset_Pin;
  HAL_GPIO_Init(GSM_Reset_Port, &GPIO_InitStruct);
#endif

  GPIO_GSM_Stat_CLK_ENABLE();
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull     = GPIO_PULLUP;
  GPIO_InitStruct.Pin       = GSM_Stat_Pin;
  HAL_GPIO_Init(GSM_Stat_Port, &GPIO_InitStruct);
  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */
/** @defgroup HAL_MSP_Private_Functions
* @{
*/

/**
* @brief UART MSP Initialization 
*        This function configures the hardware resources used in this example: 
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration  
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{ 
    GPIO_InitTypeDef  GPIO_InitStruct;

    if (huart==&UartWiFiHandle)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USARTx_TX_GPIO_CLK_ENABLE();
        USARTx_RX_GPIO_CLK_ENABLE();


        /* Enable USARTx clock */
        USARTx_CLK_ENABLE(); 
        __SYSCFG_CLK_ENABLE();

#ifdef USE_STM32F1xx_NUCLEO      
        __HAL_AFIO_REMAP_USART3_PARTIAL();
#endif

        /*##-2- Configure peripheral GPIO ##########################################*/  
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = WiFi_USART_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO) 
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = WiFi_USARTx_TX_AF;
#endif  
#ifdef USE_STM32F1xx_NUCLEO      
        GPIO_InitStruct.Pull     = GPIO_PULLUP;
#endif  
        GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

        HAL_GPIO_Init(WiFi_USART_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = WiFi_USART_RX_PIN;
#ifdef USE_STM32F1xx_NUCLEO  
        GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
#endif  
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
        GPIO_InitStruct.Alternate = WiFi_USARTx_RX_AF;
#endif

        HAL_GPIO_Init(WiFi_USART_RX_GPIO_PORT, &GPIO_InitStruct);


        /* UART RTS GPIO pin configuration  */
        GPIO_InitStruct.Pin = WiFi_USART_RTS_PIN;
#ifdef USE_STM32F1xx_NUCLEO  
        GPIO_InitStruct.Pull     = GPIO_PULLDOWN;
        GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_AF_PP;
#endif  
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
        GPIO_InitStruct.Pull     = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = WiFi_USARTx_RX_AF;
#endif

        HAL_GPIO_Init(WiFi_USART_RTS_GPIO_PORT, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for UART ########################################*/
        /* NVIC for USART */
        #if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
        HAL_NVIC_SetPriority(USARTx_IRQn, 3, 0);
        #else
        HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
        #endif
        HAL_NVIC_EnableIRQ(USARTx_IRQn);
    }
    if(huart==&UartGSMHandle)
    {
      /*##-1- Enable peripherals and GPIO Clocks #################################*/
      /* Enable GPIO TX/RX clock */
      USART_GSM_TX_GPIO_CLK_ENABLE();
      USART_GSM_RX_GPIO_CLK_ENABLE();


      /* Enable USARTx clock */
      USART_GSM_CLK_ENABLE(); 
      
      /*##-2- Configure peripheral GPIO ##########################################*/  
      /* UART TX GPIO pin configuration  */
      GPIO_InitStruct.Pin       = USART_GSM_TX_PIN;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = USART_GSM_TX_AF;

      HAL_GPIO_Init(USART_GSM_TX_GPIO_PORT, &GPIO_InitStruct);

      /* UART RX GPIO pin configuration  */
      GPIO_InitStruct.Pin = USART_GSM_RX_PIN;
      GPIO_InitStruct.Alternate = USART_GSM_RX_AF;

      HAL_GPIO_Init(USART_GSM_RX_GPIO_PORT, &GPIO_InitStruct);      
      
    }
}


/**
* @brief UART MSP De-Initialization 
*        This function frees the hardware resources used in this example:
*          - Disable the Peripheral's clock
*          - Revert GPIO and NVIC configuration to their default state
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart==&UartWiFiHandle)
    {
        /*##-1- Reset peripherals ##################################################*/
        USARTx_FORCE_RESET();
        USARTx_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(WiFi_USART_TX_GPIO_PORT, WiFi_USART_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(WiFi_USART_RX_GPIO_PORT, WiFi_USART_RX_PIN);

        /*##-3- Disable the NVIC for UART ##########################################*/
        HAL_NVIC_DisableIRQ(USARTx_IRQn);
    }
}


/**
* @brief TIM MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
* @param htim: TIM handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim==&PushTimHandle)
    {
        /* TIMx Peripheral clock enable */
        TIMp_CLK_ENABLE();
        
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIMp_IRQn, 3, 0);
        
        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIMp_IRQn);
    }
    else
    {    
        /* TIMx Peripheral clock enable */
        TIMx_CLK_ENABLE();
        
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIMx_IRQn, 3, 0);
        
        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIMx_IRQn);
    }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
