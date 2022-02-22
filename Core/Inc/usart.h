/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UartSlaveRx(void);
/* USER CODE END Prototypes */

#define IDENT_TELEGRAM 0
#define EXCHANGE_TELEGRAM 1

#define DOUT_DATA_LENGTH 56

typedef enum{
  EXT_BLOCK_IDENT     =  0,
  EXT_BLOCK_EXCHANGE  =  1
}exchangeDataStateType;

typedef union{
  uint16_t word;
  struct{
    exchangeDataStateType telegramType  : 1;  //Тип запроса: индентификация блока расширения или обмен
    uint16_t reserv                     : 15;
  }bits;
}headerType;

typedef struct{
  headerType header; 
  uint8_t reserv;
  uint16_t crc;
}identDataType;

typedef enum{
  NOT_DEF             = 0,
  INCREMENT_EXT_BLOCK = 1,
  SERIAL_EXT_BLOCK    = 2,
  SIN_COS_EXT_BLOCK   = 3,
  RS485_EXT_BLOCK     = 4,
  DIN_DOUT_EXT_BLOCK  = 5,
  AIN_AOUT_EXT_BLOCK  = 6,
  CAN_EXT_BLOCK       = 7,
  PROFIBUS_EXT_BLOCK  = 8,
  ETHER_EXT_BLOCK     = 9,
  PROFINET_EXT_BLOCK  = 10
}extBlockModeType;

typedef struct{
  extBlockModeType extBlock;
  uint16_t crc;
}identAnswType;


  //Струткура принятых от верхнего уровня данных 
typedef __packed struct{
  headerType header;
  uint8_t DoutData[DOUT_DATA_LENGTH]; //Входные данные ПЛК
  uint16_t crc;
}RxExchDataType;

  
#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

