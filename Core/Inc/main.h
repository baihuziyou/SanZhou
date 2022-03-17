/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  
	#define RECEIVEBUFFNUMBER  1024                 //定义一个宏，代表DMA接受数据的长度
	#define XDistanceMax 800                        //表示X的最远距离为800mm
	#define YDistanceMax 700											  //表示Y的最远距离为700mm
	#define ZDistanceMax 400											  //表示Z的最远距离为400mm

	
  extern uint8_t ReceiveBuff[RECEIVEBUFFNUMBER]; //DMA接受到的数据存放在这个数组中
  extern int RealReciveDateNumber;               //接受到的数据的真实长度
	
	
	extern int Instruction;            //定义从Lora模块获得的指令下达的指令是什么
	
	extern int ULimitSwithcState;      //定义向上的限位开关的状态，0x00代表限位开关没有打开，0x01代表执行机构到了限位开关的位置，也就是LED会亮
	extern int FLimitSwithcState;      //定义向后的限位开关的状态，0x00代表限位开关没有打开，0x01代表执行机构到了限位开关的位置，也就是LED会亮
	extern int RLimitSwithcState;      //定义向右的限位开关的状态，0x00代表限位开关没有打开，0x01代表执行机构到了限位开关的位置，也就是LED会亮
	
	extern int XPosition;    //定义从X,Y,Z的位置信息。移动多远记录一下
	extern int YPosition;
	extern int ZPosition;	
	
	extern GPIO_PinState XNextDir;    //定义从X,Y,Z的小一时刻的位置
	extern GPIO_PinState YNextDir;
	extern GPIO_PinState ZNextDir;	
	
	extern int XNextPosition; //定义从X,Y,Z的下一刻位置信息。
	extern int YNextPosition;
	extern int ZNextPosition;	
	
	extern uint8_t CompleteState; //完成标志位0xff代表结束
	
	extern int OneMillimeterPluse ;    //定义1mm需要走多少个脉冲

	extern int XDistance;
	extern int YDistance;
	extern int ZDistance;
	
	extern int XDistancePluse;
	extern int YDistancePluse;
	extern int ZDistancePluse;
	

	
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define F_SIGN_Pin GPIO_PIN_2
#define F_SIGN_GPIO_Port GPIOE
#define F_SIGN_EXTI_IRQn EXTI2_IRQn
#define U_SIGN_Pin GPIO_PIN_4
#define U_SIGN_GPIO_Port GPIOE
#define U_SIGN_EXTI_IRQn EXTI4_IRQn
#define FB_PU_Pin GPIO_PIN_0
#define FB_PU_GPIO_Port GPIOA
#define FB_DIR_Pin GPIO_PIN_1
#define FB_DIR_GPIO_Port GPIOA
#define LR_PU_Pin GPIO_PIN_6
#define LR_PU_GPIO_Port GPIOA
#define LR_DIR_Pin GPIO_PIN_7
#define LR_DIR_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define UD_PU_Pin GPIO_PIN_12
#define UD_PU_GPIO_Port GPIOD
#define UD_DIR_Pin GPIO_PIN_13
#define UD_DIR_GPIO_Port GPIOD
#define R_SIGN_Pin GPIO_PIN_1
#define R_SIGN_GPIO_Port GPIOE
#define R_SIGN_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
