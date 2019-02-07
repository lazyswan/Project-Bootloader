/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void readFromHost(void);
void jmpToUserApp(void);
//Prototype functoins for Bootloader 
void getVerHandler(uint8_t *rxBuffer);
void getHelpHandler(uint8_t *rxBuffer);
void getCidHandler(uint8_t *rxBuffer);
void getrdpHandler();

void goToAddrHandler();
void flashEraseHandler();
void memWrite();
void enRWProtectHandler();
void memReadHandler();
void rdSectorProtectStatusHandler();
void rdOTPHandler();
void disRWProtectHandler();
void sendNack(void);
void sendAck(uint8_t cmdCode,uint8_t bytesToFollow);





void printMsg(char* msg,...);
void delayS(uint16_t ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR_ADDRESS 0x08008000U
/* USER CODE END Private defines */



//CRC
#define CRC_SUCCESS 0
#define CRC_FAIL    1

//Bootloader commands:
#define VERSION             0x20
#define NACK                0X7F
#define ACK                 0XA5

//#define  <command name >  <command_code>

//This command is used to read the bootloader version from the MCU
#define BL_GET_VER                0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP               0x52

//This command is used to read the MCU chip identification number
#define BL_GET_CID                0x53

//This command is used to read the FLASH Read Protection level.
#define BL_GET_RDP_STATUS         0x54

//This command is used to jump bootloader to specified address.
#define BL_GO_TO_ADDR             0x55

//This command is used to mass erase or sector erase of the user flash .
#define BL_FLASH_ERASE            0x56

//This command is used to write data in to different memories of the MCU
#define BL_MEM_WRITE              0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
#define BL_EN_RW_PROTECT          0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ               0x59

//This command is used to read all the sector protection status.
#define BL_READ_SECTOR_P_STATUS   0x5A


//This command is used to read the OTP contents.
#define BL_OTP_READ               0x5B


//This command is used disable all sector read/write protection 
#define BL_DIS_R_W_PROTECT        0x5C

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
