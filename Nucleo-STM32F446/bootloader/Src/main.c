/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

#include <stdarg.h>//va list 
#include <string.h>
#include <stdint.h>

/* Includes ------------------------------------------------------------------*/
#include "main.h"

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);

char msg[]="Message printed over UART2\n";


//Bootloader
#define COMPORT &huart2
#define BUFFER_LENGTH 200
uint8_t  rxBuffer[BUFFER_LENGTH];
uint8_t  supportedCommands[]={
	BL_GET_VER,
	BL_GET_HELP,
	BL_GET_CID,
	BL_FLASH_ERASE,
	BL_MEM_WRITE,
	};
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
 
  while (1)
  {
		
		if(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)==GPIO_PIN_RESET){
			printMsg("Button Pressed\n");
			delayS(1000);			
			readFromHost();//Bootloader Function:
		}
		else{
			jmpToUserApp();//Jumps to User Application
			printMsg("Button Not Pressed\n");
			delayS(1000);
				
		}
    
  }
 
}
void printMsg(char* msg,...){
char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, msg);
	vsprintf(str, msg,args);
	HAL_UART_Transmit(&huart2,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
	va_end(args);
	
	
}
void delayS(uint16_t ms){
	uint32_t currentTick=HAL_GetTick();
		// BUSY WAITING
			while(HAL_GetTick()<=currentTick+ms);
}
void readFromHost(){
	uint8_t rcvLength=0;
	while(1){
		memset(rxBuffer,0,BUFFER_LENGTH);//Reinitialsed the Buffer for new data;
		HAL_UART_Receive(COMPORT,rxBuffer,1,HAL_MAX_DELAY);
		rcvLength=rxBuffer[0];//Length of Bytes to follow:
		HAL_UART_Receive(COMPORT,&rxBuffer[1],rcvLength,HAL_MAX_DELAY);
		
		switch(rxBuffer[1]){
			case BL_GET_VER:
				getVerHandler(rxBuffer);
				break;
			case BL_GET_HELP:
				getHelpHandler(rxBuffer);
				break;
			case BL_GET_CID:
				getCidHandler(rxBuffer);
				break;
			case BL_FLASH_ERASE:
				flashEraseHandler(rxBuffer);
				break;
			case BL_MEM_WRITE:
				memWrite(rxBuffer);
				break;
			default:
				//printMsg("BL:MSG::Invalid Command--\n");
				break;
			
		}
		
	}
	
	
}

//Function Jumps to ResetHandler of User Application @ location 0x0800 8000
void jmpToUserApp(){
	void (*app_reset_handler)(void);//pointer to hold the reset handler
	//first address @ 0x0800_8000 hold MSP
	uint32_t mspVal= *(volatile uint32_t*)FLASH_SECTOR_ADDRESS;
	__set_MSP(mspVal);
	//second address is address of reset handler
	uint32_t resetHandler= *(volatile uint32_t*)(FLASH_SECTOR_ADDRESS+4);
	app_reset_handler = (void *)resetHandler;
	//Jumping to User Application;
	app_reset_handler();
	
}
//************************Bootloader Command Handlers Definatons: ***************************

void sendNack(void){
	
	uint8_t nackBit=NACK;
	HAL_UART_Transmit(COMPORT,&nackBit,1,HAL_MAX_DELAY);
	
}

void sendAck(uint8_t cmdCode,uint8_t bytesToFollow){
	//ACK is TWO Bytes
	uint8_t ackBuff[2];
	ackBuff[0]=ACK;
	ackBuff[1]=bytesToFollow;
	HAL_UART_Transmit(COMPORT,ackBuff,2,HAL_MAX_DELAY);
	
}

uint8_t verifyCRC(uint8_t *pData,uint32_t len,uint32_t crcHost){
	uint32_t uwCRCValue=0xff;
	for(uint32_t i=0;i<len;i++){
		uint32_t data=pData[i];
		uwCRCValue=HAL_CRC_Accumulate(&hcrc,&data,1);
	}
	//RESEET CRC CALCULATOIN UNIT
	__HAL_CRC_DR_RESET(&hcrc);
	if(uwCRCValue==crcHost)
		return CRC_SUCCESS;
	else return CRC_FAIL;
}
void getVerHandler(uint8_t *rxBuffer){
	//printMsg("BL::Getting Version");
	uint8_t pck_length=rxBuffer[0]+1;
	uint32_t hostCRC=*(uint32_t *)(rxBuffer+pck_length-4);
	
	if(!verifyCRC(&rxBuffer[0],pck_length-4,hostCRC)){
		sendAck(rxBuffer[1],1);
		uint8_t version=VERSION;
		HAL_UART_Transmit(COMPORT,&version,1,HAL_MAX_DELAY);
	}
	
	else{
		sendNack();
		
	}
	
}
void getHelpHandler(uint8_t *rxBuffer){
	uint8_t pck_length=rxBuffer[0]+1;
	uint32_t hostCRC=*(uint32_t *)(rxBuffer+pck_length-4);
	
	if(!verifyCRC(&rxBuffer[0],pck_length-4,hostCRC)){
		sendAck(rxBuffer[1],sizeof(supportedCommands));
		//uint8_t version=VERSION;
		HAL_UART_Transmit(COMPORT,supportedCommands,sizeof(supportedCommands),HAL_MAX_DELAY);
	}
	
	else{
		sendNack();		
	}
	
	
}

void getCidHandler(uint8_t *rxBuffer){
	uint8_t pck_length=rxBuffer[0]+1;
	uint32_t hostCRC=*(uint32_t *)(rxBuffer+pck_length-4);
	
	if(!verifyCRC(&rxBuffer[0],pck_length-4,hostCRC)){
		uint16_t cid=(uint16_t)((DBGMCU->IDCODE) & 0X0FFF);
		sendAck(rxBuffer[1],sizeof(cid));
		//uint8_t version=VERSION;
		
		HAL_UART_Transmit(COMPORT,(uint8_t *)&cid,sizeof(cid),HAL_MAX_DELAY);
	}
	
	else{
		sendNack();		
	}
	
}

void flashEraseHandler(uint8_t *rxBuffer){
	/*To Erase the Flash is to Write 0xFF;
	0xAB->0xCD | not allowed
	0xAB->0xFF and then 0xFF->0xAB;	
	*/
	uint8_t eraseStatus=0x00;
	uint8_t pck_length=rxBuffer[0]+1;
	uint32_t hostCRC=*((uint32_t *)(rxBuffer+pck_length-4));
	
	if(!verifyCRC(&rxBuffer[0],pck_length-4,hostCRC)){
		sendAck(rxBuffer[1],1);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1);
		eraseStatus=eraseFlash(rxBuffer[2], rxBuffer[3]);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);
		HAL_UART_Transmit(COMPORT,(uint8_t *)&eraseStatus,sizeof(eraseStatus),HAL_MAX_DELAY);
	}
	
	else{
		sendNack();
		
	}	
	
}

uint8_t eraseFlash(uint8_t sectorNum, uint8_t numOfSectors){
	FLASH_EraseInitTypeDef flash_handler;
	uint32_t sectorError;
	HAL_StatusTypeDef status;
	if(numOfSectors>8){
		return INVALID_SECTOR;
	}
	if(sectorNum<=7 || sectorNum == 0xFF ){
		if(sectorNum==((uint8_t) 0xFF)){
		flash_handler.TypeErase=FLASH_TYPEERASE_MASSERASE;
		}		
		else{
			uint8_t remainingSector=8-sectorNum;
			if(numOfSectors>remainingSector){
				numOfSectors=remainingSector;
			}
			flash_handler.TypeErase=FLASH_TYPEERASE_SECTORS;
			flash_handler.Sector=sectorNum;
			flash_handler.NbSectors=numOfSectors;
		}
			flash_handler.Banks=FLASH_BANK_1;
			HAL_FLASH_Unlock();
			flash_handler.VoltageRange=FLASH_VOLTAGE_RANGE_3;
			status=(uint8_t)HAL_FLASHEx_Erase(&flash_handler,&sectorError);			
			HAL_FLASH_Lock();
			return status;		
		
	}
	//flash_handler.
	
	
}
void memWrite(uint8_t *rxBuffer){
	uint8_t pck_length=rxBuffer[0]+1;
	uint32_t hostCRC=*(uint32_t *)(rxBuffer+pck_length-4);
	
	if(!verifyCRC(&rxBuffer[0],pck_length-4,hostCRC)){
		sendAck(rxBuffer[1],sizeof(supportedCommands));
		//uint8_t version=VERSION;
		HAL_UART_Transmit(COMPORT,supportedCommands,sizeof(supportedCommands),HAL_MAX_DELAY);
	}
	
	else{
		sendNack();		
	}
	
	
}



//************************Bootloader Command Handlers Ends ***************************

























void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
