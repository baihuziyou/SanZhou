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
  
	#define RECEIVEBUFFNUMBER  1024                 //����һ���꣬����DMA�������ݵĳ���
	#define XDistanceMax 800                        //��ʾX����Զ����Ϊ800mm
	#define YDistanceMax 700											  //��ʾY����Զ����Ϊ700mm
	#define ZDistanceMax 400											  //��ʾZ����Զ����Ϊ400mm

	
  extern uint8_t ReceiveBuff[RECEIVEBUFFNUMBER]; //DMA���ܵ������ݴ�������������
  extern int RealReciveDateNumber;               //���ܵ������ݵ���ʵ����
	
	
	extern int Instruction;            //�����Loraģ���õ�ָ���´��ָ����ʲô
	
	extern int ULimitSwithcState;      //�������ϵ���λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����
	extern int FLimitSwithcState;      //����������λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����
	extern int RLimitSwithcState;      //�������ҵ���λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����
	
	extern int XPosition;    //�����X,Y,Z��λ����Ϣ���ƶ���Զ��¼һ��
	extern int YPosition;
	extern int ZPosition;	
	
	extern GPIO_PinState XNextDir;    //�����X,Y,Z��Сһʱ�̵�λ��
	extern GPIO_PinState YNextDir;
	extern GPIO_PinState ZNextDir;	
	
	extern int XNextPosition; //�����X,Y,Z����һ��λ����Ϣ��
	extern int YNextPosition;
	extern int ZNextPosition;	
	
	extern uint8_t CompleteState; //��ɱ�־λ0xff�������
	
	extern int OneMillimeterPluse ;    //����1mm��Ҫ�߶��ٸ�����

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
