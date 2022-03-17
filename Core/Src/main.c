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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t ReceiveBuff[RECEIVEBUFFNUMBER]; //DMA���ܵ������ݴ�������������
int RealReciveDateNumber;               //���ܵ������ݵ���ʵ����

uint8_t CompleteState = 0Xff;      //ָ�������ߵ�λ����ɱ�־λ

int OneMillimeterPluse = 320 ;   //��ʾ1mm��Ҫ320���������

int Instruction =0x55;   //ָ����ʲô�����㣬λ��ģʽ�ƶ�������ģʽ�ƶ�,��������

//����������λ���ص�״̬�����ⲿ�ж��и�����ֵ
int ULimitSwithcState = 0x00;   //�������ϵ���λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����
int FLimitSwithcState = 0x00;   //����������λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����
int RLimitSwithcState = 0x00;   //�������ҵ���λ���ص�״̬��0x00������λ����û�д򿪣�0x01����ִ�л���������λ���ص�λ�ã�Ҳ����LED����

 //����������ߵľ���
int XDistance = 0;         
int YDistance = 0;
int ZDistance = 0;


//�����X,Y,Z��λ����Ϣ���ƶ���ʱ���¼
int XPosition = 0;    
int YPosition = 0;
int ZPosition = 0;	

//�����ƶ�����

GPIO_PinState XNextDir;  
GPIO_PinState YNextDir;
GPIO_PinState ZNextDir;
              
int XNextPosition; //�����X,Y,Z����һ��λ����Ϣ��
int YNextPosition;
int ZNextPosition;	
	


//�����������Ҫ�ߵ��������
int XDistancePluse = 0;          
int YDistancePluse = 0;
int ZDistancePluse = 0;

//������������ķ����������ȫ�ֱ�����ֻ����main.c��ʹ��
//GPIO_PinState XDir = GPIO_PIN_SET;
//GPIO_PinState YDir = GPIO_PIN_SET;
//GPIO_PinState ZDir = GPIO_PIN_SET;




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


void GoForwardBack(void);   //������ǰ����ߵĺ���
void GoLeftRight(void);
void GoUpDown(void);

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	

	//����Ĵ�����Щ�������ܷ���begin1�У���Ϊ��û�д��ڳ�ʼ���Ͳ�����
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,ReceiveBuff,RECEIVEBUFFNUMBER);

	//Time3,TIME4,TIME5������APB1�ϣ��ٶ�Ϊ84Mhz
	//����ʱ��3.4.5��ʱ��
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start(&htim5);
	
	

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(Instruction == 0x50 )  //����
		{
			Instruction = 0x55 ;
			GoForwardBack();
			GoLeftRight();
			GoUpDown();
		}
		
		else if(Instruction == 0x51)
		{
			Instruction = 0x55;    //������������ 1.����������0��������λ����û�д��� �� 2.����ƶ��������0����λ���ش�����reset�������������ƶ�����Ϊ������
			GoForwardBack();
			GoLeftRight();
			GoUpDown();
		}
		
		
		else if(Instruction == 0x52)
		{
			Instruction = 0x55;    //������������ 1.����������0��������λ����û�д��� �� 2.����ƶ��������0����λ���ش�����reset�������������ƶ�����Ϊ������
			GoForwardBack();
			GoLeftRight();
			GoUpDown();
		}
		
		else 
		{

		}
			
		
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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


//��ǰ���Ҳ����X�ᣬ��ǰ�����������ӵ��˶�ʱ��5��CH1ͨ����Ҳ����PA0
void GoForwardBack()
{
	 XDistancePluse = XDistance * OneMillimeterPluse; 	
	 if(XDistancePluse != 0)
	 {
		 	HAL_GPIO_WritePin(FB_DIR_GPIO_Port,FB_DIR_Pin,XNextDir);
			HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_1);    //����PWM�źţ�Ҳ�����õ��ת���ˣ������ж����жϵ���ߵ�λ�˺󣬾�ֹͣ
	 }

}

//��������Ҳ����Y�ᣬ��ǰ�����������ӵ��˶�ʱ��3��CH1ͨ����Ҳ����PA6
void GoLeftRight()
{

		YDistancePluse = YDistance * OneMillimeterPluse; 
		if(YDistancePluse != 0)
		{
			HAL_GPIO_WritePin(LR_DIR_GPIO_Port,LR_DIR_Pin,YNextDir);
			HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);    //����PWM�źţ�Ҳ�����õ��ת���ˣ������ж����жϵ���ߵ�λ�˺󣬾�ֹͣ
		}

}

//��������Ҳ����Z�ᣬ��ǰ�����������ӵ��˶�ʱ��4��CH1ͨ����Ҳ����PD12
void GoUpDown()
{

	ZDistancePluse = ZDistance * OneMillimeterPluse; 
	if(ZDistancePluse != 0)
	{
			HAL_GPIO_WritePin(UD_DIR_GPIO_Port,UD_DIR_Pin,ZNextDir);
			HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);    //����PWM�źţ�Ҳ�����õ��ת���ˣ������ж����жϵ���ߵ�λ�˺󣬾�ֹͣ 
	}
}





//һ��PWM������ɺ�ͽ�������ж�
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) 
{
	
	if(htim == &htim3)  //y����
	{
			//���ж���λ������û�д���
			RLimitSwithcState = HAL_GPIO_ReadPin(R_SIGN_GPIO_Port,R_SIGN_Pin) ;
	    if(RLimitSwithcState == 0  && YNextDir == GPIO_PIN_RESET) 
			{
				YDistancePluse = 0;
			}
		
		  static int TemporaryYPluse = 0;
			if(TemporaryYPluse++ >= YDistancePluse)  
			{
				TemporaryYPluse =0;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); 
				YPosition=YNextPosition;  //����XYZ���µ�λ��
				HAL_UART_Transmit(&huart1,&CompleteState,1,10);
			}
	}
	
	else if(htim == &htim4) //z����
	{
		ULimitSwithcState = HAL_GPIO_ReadPin(U_SIGN_GPIO_Port,U_SIGN_Pin) ;
		if(ULimitSwithcState == 0  && ZNextDir == GPIO_PIN_RESET) 
		{
			ZDistancePluse = 0;
		}
		
		static int TemporaryZPluse=0;
		if(TemporaryZPluse++ >= ZDistancePluse)
		{
			TemporaryZPluse=0;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); 
			ZPosition=ZNextPosition;
			HAL_UART_Transmit(&huart1,&CompleteState,1,10);
		}
	}

	else if(htim == &htim5)   //x����
	{
		FLimitSwithcState = HAL_GPIO_ReadPin(F_SIGN_GPIO_Port,F_SIGN_Pin) ;
	  if(FLimitSwithcState == 0  && XNextDir == GPIO_PIN_RESET) 
		{
			XDistancePluse = 0;
		}
		
		static int TemporaryXPluse=0;
		if(TemporaryXPluse++ >= XDistancePluse)
		{
			TemporaryXPluse = 0;
			HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1); 
			XPosition=XNextPosition;
			HAL_UART_Transmit(&huart1,&CompleteState,1,10);
		}
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
