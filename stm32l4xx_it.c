/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"

#include "main.h"
#include "stm32l4xx_it.h"
#include "wifi_module.h"
#include "stm32_spwf_wifi.h"
#include "string.h"   
#include "ring_buffer.h" 
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "GSM.h"
/** @defgroup STM32xx_IT_Private_Variables
* @{
*/

extern UART_HandleTypeDef UartWiFiHandle,UartMsgHandle;
extern TIM_HandleTypeDef  TimHandle, PushTimHandle;

/** @addtogroup STM32xx_HAL_Examples
* @{
*/
extern wifi_bool Timer_Running;
extern wifi_bool Standby_Timer_Running;
extern wifi_bool AT_Cmd_Processing;
extern uint32_t tickcount;
extern uint32_t standby_time;
extern wifi_bool Deep_Sleep_Timer;
extern wifi_bool Deep_Sleep_Enabled;
extern wifi_bool Deep_Sleep_Callback;
extern uint32_t sleep_count;
extern volatile uint32_t BpushButtonState;

/** @addtogroup Templates
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void USARTx_IRQHandler(void);    
void USARTx_PRINT_IRQHandler(void);
void USARTx_EXTI_IRQHandler(void);
void TIMx_IRQHandler(void);
void TIMp_IRQHandler(void);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
        BSP_LED_On(LED2); 
    }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
        BSP_LED_On(LED2); 
    }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
        BSP_LED_On(LED2); 
    }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
        BSP_LED_On(LED2); 
    }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
//void SVC_Handler(void)
//{
//  BSP_LED_On(LED2); 
//}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
    BSP_LED_On(LED2); 
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
//void PendSV_Handler(void)
//{
//}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
    TIMx counter clock equal to 10000 Hz . With a period of 1K. so it is running at 10Hz. 100ms/.1s
*/
void TIMx_IRQHandler(void)									//TIM3_IRQHandler
{
    HAL_TIM_IRQHandler(&TimHandle);

}

/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
*/
void TIMp_IRQHandler(void)								//TIM2_IRQHandler : NOT used
{
    HAL_TIM_IRQHandler(&PushTimHandle);
}

/**
* @brief  This function handles SysTick Handler.
*/
void SysTick_Handler(void) //1 ms interrupt @ 84 MHz core. Running at 1000Hz
{
    HAL_IncTick();
//    osSystickHandler();
//    Wifi_SysTick_Isr();
    gsm1msPing();
}

/**
* @brief  This function GPIO EXTI Callback.
* @param  GPIO_Pin number of the GPIO generating the EXTI IRQ
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    BpushButtonState = 1;
}


/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
    Wifi_TIM_Handler(htim);
}


/**
* @brief  HAL_UART_RxCpltCallback
*         Rx Transfer completed callback
* @param  UartHandleArg: UART handle 
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
    WiFi_HAL_UART_RxCpltCallback(UartHandleArg);
}

/**
* @brief  HAL_UART_TxCpltCallback
*         Tx Transfer completed callback
* @param  UartHandleArg: UART handle 
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
    WiFi_HAL_UART_TxCpltCallback(UartHandleArg);
}

/**
* @brief  UART error callbacks
* @param  UartHandle: UART handle
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    WiFi_HAL_UART_ErrorCallback(UartHandle);
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/**
* @brief  This function handles USARTx Handler.
* @param  None
* @retval None
*/
void USARTx_IRQHandler(void)													//USART1_IRQHandler
{
    HAL_UART_IRQHandler(&UartWiFiHandle);
    
}

/**
* @brief  This function handles USARTx vcom Handler.
* @param  None
* @retval None
*/
#ifdef USART_PRINT_MSG
void USARTx_PRINT_IRQHandler(void)										//USART2_IRQHandler
{
    HAL_UART_IRQHandler(&UartMsgHandle);
}
#endif
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
