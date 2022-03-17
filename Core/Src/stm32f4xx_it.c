/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET )
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);  
		HAL_UART_DMAStop(&huart1);//这个不关闭会出现接受不全的现象等。
		RealReciveDateNumber = RECEIVEBUFFNUMBER - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取真真实的数据长度
	
		HAL_UART_Transmit_DMA(&huart1,ReceiveBuff,RealReciveDateNumber);
		
		
		if(ReceiveBuff[0] == 0x50 && ReceiveBuff[11] == 0x60)   //第一个为50，第11个为60代表这是一帧数据
		{
			
				XNextPosition = ReceiveBuff[3]*100+ReceiveBuff[4];
				YNextPosition = ReceiveBuff[6]*100+ReceiveBuff[7];
				ZNextPosition = ReceiveBuff[9]*100+ReceiveBuff[10];
		
			
			if(ReceiveBuff[1] == 0x50 )    //第一个数据是0x50代表让三轴复位归0
			{
								
				XNextDir = GPIO_PIN_RESET;
				YNextDir = GPIO_PIN_RESET;
				ZNextDir = GPIO_PIN_RESET;
				
				XDistance = 1000 ;
				YDistance = 1000 ;
				ZDistance = 1000 ;
				
				Instruction = 0x50; 
			}
			
			
			else if(ReceiveBuff[1] == 0x51)   //第一个数据是0x51代表让电机移动到指定的位置，读取对应的XYZ需要走多少参数
			{
				//根据输入的目标位置先获取每个电机应该往那边转动
				XNextDir = (XNextPosition - XPosition > 0 ) ? GPIO_PIN_SET   : GPIO_PIN_RESET  ;
				YNextDir = (YNextPosition - YPosition > 0 ) ? GPIO_PIN_SET   : GPIO_PIN_RESET  ;
				ZNextDir = (ZNextPosition - ZPosition > 0 ) ? GPIO_PIN_SET   : GPIO_PIN_RESET  ;
				

				//计算XYZ对应要走多远   ,,,,,,,,,,,,,,,,这里确保他>0试试
				XDistance = XNextPosition - XPosition ;
				YDistance = YNextPosition - YPosition ;
				ZDistance = ZNextPosition - ZPosition ;
				
				if(XDistance == 0x00) HAL_UART_Transmit(&huart1,&CompleteState,1,100);
				if(XDistance == 0x00) HAL_UART_Transmit(&huart1,&CompleteState,1,100);
				if(XDistance == 0x00) HAL_UART_Transmit(&huart1,&CompleteState,1,100);
				
				XDistance = ( ( XNextPosition - XPosition ) > 0 ) ? XDistance*1: XDistance * (-1);
				YDistance = ( ( YNextPosition - YPosition ) > 0 ) ? YDistance*1: YDistance * (-1);
 				ZDistance = ( ( ZNextPosition - ZPosition ) > 0 ) ? ZDistance*1: ZDistance * (-1);				
				
				Instruction = 0x51;  //指定移动的距离以及方向
			}
			
			
			else if(ReceiveBuff[1] == 0x52)     //第一个数据是0x52代表让三轴开启点动模式
			{
		  	//点动模式位置有所不同
				
				//可能有问题，没有考虑+，-
				
				
				//根据输入的目标位置先获取每个电机应该往那边转动
				XNextDir = (ReceiveBuff[2] == 0x00) ? GPIO_PIN_SET   : GPIO_PIN_RESET ;   //X往远端走，X增大, Reset
				YNextDir = (ReceiveBuff[5] == 0x00) ? GPIO_PIN_SET   : GPIO_PIN_RESET;    //Y往远端走，Y增大，Set
				ZNextDir = (ReceiveBuff[8] == 0x00) ? GPIO_PIN_SET   : GPIO_PIN_RESET ;   //Z往远端走，Z增大，Reset
				
				XDistance = ReceiveBuff[3]*100+ReceiveBuff[4] ;
				YDistance = ReceiveBuff[6]*100+ReceiveBuff[7] ;
				ZDistance = ReceiveBuff[9]*100+ReceiveBuff[10];
				
				Instruction = 0x52;  //点动模式，按一下走一下
			}
			
				
			
			else   //否则的话就是让电机不要动
			{
				Instruction = 0x60;
			}
			
		
		}
		

		memset(ReceiveBuff,0,RealReciveDateNumber);   //清除数据，要包含头文件string.h
		HAL_UART_Receive_DMA(&huart1, ReceiveBuff, RECEIVEBUFFNUMBER);     //重新打开dma接收
		
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
