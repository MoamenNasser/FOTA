/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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


uint8_t INTflag=0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
	MX_CRC_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t data_packet[1031]={0};
	uint8_t data_array[1024]={0};
	uint32_t CRC_Value;
	uint8_t cmd_packet [6]={0};
	/* search file at server */

	uint32_t file_size;

	GSM_Get_File_From_Server();

	file_size= GSM_GET_FILE_SIZE();

	GSM_TO_STM_TO_EEPROM_FILE_SEND( file_size);

	/* transmitting the first packet (command Packet)*/

	cmd_packet[0]=5;
	cmd_packet[1]=0x31;
	for (uint32_t i=0;i<2;i++)
	{
		uint32_t i_data=cmd_packet[i];
		CRC_Value=HAL_CRC_Accumulate(&hcrc, &i_data, 1);

	}

	//first_packet=first_packet+2;
	uint8_t *ptr=cmd_packet;
	ptr+=2;
	*((uint32_t *)ptr) = CRC_Value;

	while(INTflag!=1)
	{

	}

	HAL_Delay(2000);
	HAL_UART_Transmit(&huart2, cmd_packet, sizeof(cmd_packet), HAL_MAX_DELAY);

	/* getting the data from the EEPROM and sending it to the other STM32*/

	uint8_t temp=ceil(file_size/1024.0);
	for(int counter=0;counter<temp;counter++)
	{
		for(int counter_1=0;counter_1<16;counter_1++)
		{

			HAL_I2C_Mem_Read(&hi2c2, 0XA0, 64*counter_1+counter*1024, 2, &data_array[64*counter_1], 64, 1000);
			HAL_Delay(10);
		}

		/* constructing the data packet to send it using UART2*/
		/* c means we send data array of 1024 */
		data_packet [0]='c';
		/* adjusting the flag*/
		if(counter==temp-1)
		{
			data_packet[1025]=1;
			for(uint16_t count=file_size-counter*1024;count<1024;count++)
			{
				data_array[count]=0xFF;
			}
		}
		else
		{
			data_packet[1025]=0;
		}
		/*getting the data array into the data packet*/
		for(int i=1;i<1025;i++)
		{
			data_packet[i]=data_array[i-1];
		}

		/* constructing the CRC */
		CRC_Value=0;
		for (uint32_t i=0;i<1026;i++)
		{
			uint32_t i_data=data_packet[i];
			CRC_Value=HAL_CRC_Accumulate(&hcrc, &i_data, 1);

		}

		//first_packet=first_packet+2;
		ptr=data_packet;
		ptr+=1026;
		*((uint32_t *)ptr)=CRC_Value;

		while(INTflag!=counter+2)
		{

		}

		HAL_Delay(200);
		HAL_UART_Transmit(&huart2, data_packet, sizeof(data_packet), HAL_MAX_DELAY);
		HAL_Delay(10);
	}


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_12)
	{
		INTflag++;
	}
}

void GSM_Get_File_From_Server()
{
	uint8_t cmd[70];
	sprintf(cmd,"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	sprintf(cmd,"AT+SAPBR=3,1,\"APN\",\"etisalat\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	sprintf(cmd,"AT+SAPBR=1,1\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(2000);
	sprintf(cmd,"AT+FTPSERV=\"199.102.48.28\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(2000);
	sprintf(cmd,"AT+FTPPORT=\"21\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	sprintf(cmd,"AT+FTPUN=\"fota22\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	sprintf(cmd,"AT+FTPPW=\"FOTA123456\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	/* name file */HAL_Delay(200);
	sprintf(cmd,"AT+FTPGETNAME=\"ss.bin\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	sprintf(cmd,"AT+FTPGETPATH=\"/\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);

	/* save file in GSM with name "file3.txt"*/
	sprintf(cmd,"AT+FTPGETTOFS=0,\"final.bin\"\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(40000);
	/*
	sprintf(cmd,"AT+SAPBR=0,1\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_Delay(200);
	 */
}
int GSM_GET_FILE_SIZE()
{
	/* this function use AT command of GSM to get the size of file */

	int file_size=0;

	char cmd[70];
	char cmd_rx[70];
	/* AT command to get file size */
	sprintf(cmd,"AT+FSFLSIZE=C:\\User\\FTP\\final.bin\n");
	HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
	/* Receiving the output of gsm to get the size of the file */
	HAL_UART_Receive(&huart1, cmd_rx, 70, 1000);
	/* define this counter to know the length of the file in digits */
	char counter_len=0;
	/* This loop is for counting the length and construct the array of the file size*/
	for (int i =47 ;i<70;i++)
	{
		counter_len++;
		cmd_rx[i-47]=cmd_rx[i];
		if(cmd_rx[i]=='\r')
		{
			cmd_rx[i]='\0';
			break;
		}

	}
	/* set the rest of the array to null to convert to it later into integer vlaue*/
	for (int i =counter_len;i<70;i++)
	{
		cmd_rx[i]='\0';
	}

	/* convert sting to char */
	file_size=atoi(cmd_rx);
	return file_size;
}
void GSM_TO_STM_TO_EEPROM_FILE_SEND(int file_size)
{

	int counter =0;

	for (int i=0;i<ceil((file_size/1024.0));i++)
	{
		/* this array just to save AT command */
		char cmd[70]={0};

		/* this array to save 1k byte of file data*/
		/*why we initialization here not above loop to make all elements zeros */
		/* at the begging of every iteration */
		char RX_arr[1070]={0};
		sprintf(cmd,"AT+FSREAD=C:\\User\\FTP\\file1.bin,1,1024,%d\n",i*1024);
		HAL_UART_Transmit(&huart1,cmd, strlen(cmd), HAL_MAX_DELAY);
		if(i==0)
		{
			HAL_UART_Receive(&huart1, RX_arr, 1070, 1000);
		}
		/* check RX_arr index 50 if it equal '\0' or not */
		/* from that we can know is file is received or not */
		/*we choose index 50 dependent on debug */

		while (RX_arr[50]=='\0')
		{
			HAL_UART_Receive(&huart1, RX_arr, 1070, 1000);
		}
		counter=0;

		/* to count how many element useless in RX_arr */
		for (int k =0;k<1070;k++)
		{
			counter++;
			if((RX_arr[k]=='\r')&&(RX_arr[k+1]=='\n'))
			{
				break;
			}

		}

		/* to adjust RX_arr and remove all external byte coming just from GSM */
		/* doesnot belong to file data */
		for (int l =0;l<1024;l++)
		{
			RX_arr[l]=RX_arr[counter+1+l];

		}

		/* this part for the last iteration in file */
		/* because the reminder of file not equal 1024 byte we do that  */

		if(i==file_size/1024)
		{
			/* to save element by element in RX_arr to send it */
			/*byte by byte */
			//char retr[2];
			/*last iteration */

			/*
			for (int j=0;j<file_size-i*1024;j++)
			{
				retr[0]=RX_arr[j];
				HAL_UART_Transmit(&huart1,retr, 1, 1000);
				HAL_Delay(1);
			}
			 */
			uint16_t Reminder=file_size-i*1024;

			for(int counter_1=0;counter_1<ceil(Reminder/64.0);counter_1++)
			{
				if(counter_1==ceil(Reminder/64.0)-1)
				{
					HAL_I2C_Mem_Write(&hi2c2, 0XA0, 64*counter_1+1024*i, 2, &RX_arr[64*counter_1], Reminder%64, 1000);
					HAL_Delay(10);
					break;
				}
				HAL_I2C_Mem_Write(&hi2c2, 0XA0, 64*counter_1+1024*i, 2, &RX_arr[64*counter_1], 64, 1000);
				HAL_Delay(10);
			}
			break;

		}



		/* send1k to eeprom */
		/* i2c will be used insted of usart  */
		//HAL_UART_Transmit(&huart1,RX_arr,1024, HAL_MAX_DELAY);

		/*  HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
		 uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
		 */
		for(int counter_1=0;counter_1<16;counter_1++)
		{

			HAL_I2C_Mem_Write(&hi2c2, 0XA0, 64*counter_1+1024*i, 2, &RX_arr[64*counter_1], 64, 1000);
			HAL_Delay(10);
		}

	}
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

