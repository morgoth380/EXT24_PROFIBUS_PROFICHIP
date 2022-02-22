/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "crc.h"
#include "platform.h"
#include "DpAppl.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

extern DP_APPL_STRUC  sDpAppl;
uint8_t TxBuf[128] = {0};    
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
void UartSlaveRx();
uint8_t rxUartBuf[128];   
uint8_t txUartBuf[128];
void errIndGlowOff(void);

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 3000000;
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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }  
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}


/**
  * @brief  
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t Crc;     //Принятая контрольная сумма идентификационной телеграммы
  uint16_t CrcRx;   //Рассчитанная контрольная сумма идентификационной телеграммы
  uint16_t dataLength;
  uint16_t telegramType;
  uint16_t TxDataLen;
  identDataType  *idRxDataPnt;
  RxExchDataType *RxDataPnt;
  identAnswType  *idTxDataPnt;
  static exchangeDataStateType exchangeDataState = EXT_BLOCK_IDENT;
   
  switch(exchangeDataState){
  case EXT_BLOCK_IDENT:
    idRxDataPnt = (identDataType *)huart->pRxBuffPtr;
    dataLength = sizeof(identDataType) - sizeof(idRxDataPnt->crc);
    Crc = GetCrc(idRxDataPnt, dataLength);
    CrcRx = idRxDataPnt->crc;
    
    if(Crc == CrcRx){
      telegramType = idRxDataPnt->header.bits.telegramType;
      errIndGlowOff(); //Отключение индикации MK_ERR
      if(telegramType == IDENT_TELEGRAM){
        idTxDataPnt = (identAnswType *)&TxBuf[0];
        idTxDataPnt->extBlock = PROFIBUS_EXT_BLOCK; //код блока расширения
        TxDataLen = sizeof(identAnswType) - sizeof(idTxDataPnt->crc);
        idTxDataPnt->crc = GetCrc((unsigned char *)(huart->pTxBuffPtr), TxDataLen);
        HAL_UART_Transmit_DMA(huart, (uint8_t *)&TxBuf[0],  sizeof(identAnswType));
      }else{
        exchangeDataState = EXT_BLOCK_EXCHANGE;
      }
    }else{ //Если crc не совпало, то интерпретируем телеграмму как телеграмму обмена
        RxDataPnt = (RxExchDataType *)huart->pRxBuffPtr;
        Crc = GetCrc(RxDataPnt, sizeof(RxExchDataType) - sizeof(RxDataPnt->crc));
        CrcRx = RxDataPnt->crc;
        if(Crc == CrcRx){
          errIndGlowOff(); //Отключение индикации MK_ERR
          telegramType = RxDataPnt->header.bits.telegramType;
          if(telegramType == EXCHANGE_TELEGRAM){
            exchangeDataState = EXT_BLOCK_EXCHANGE;
          }
        }
    }
      
    break;
  case EXT_BLOCK_EXCHANGE: //Состояние обмена
    //Проверим, является ли принятая телеграмма телеграммой идентификации
    idRxDataPnt = (identDataType *)huart->pRxBuffPtr;
    dataLength = sizeof(identDataType) - sizeof(idRxDataPnt->crc);
    Crc = GetCrc(idRxDataPnt, dataLength);
    CrcRx = idRxDataPnt->crc;
    if(Crc == CrcRx){
      errIndGlowOff(); //Отключение индикации MK_ERR
      telegramType = idRxDataPnt->header.bits.telegramType;
      if(telegramType == IDENT_TELEGRAM){
         exchangeDataState = EXT_BLOCK_IDENT; //Возврат в состояние идентификации
      }
    }else{ //Является ли телеграмма телеграммой обмена
      RxDataPnt = (RxExchDataType *)huart->pRxBuffPtr;
      Crc = GetCrc(RxDataPnt, sizeof(RxExchDataType) - sizeof(RxDataPnt->crc));
      CrcRx = RxDataPnt->crc;
      if(Crc == CrcRx){
        errIndGlowOff();
        //Тут обрабатываем массив данных от CP24
        memcpy(&sDpAppl.abDpInputData[0], &RxDataPnt->DoutData[0], sizeof(RxDataPnt->DoutData));
      }
    }
    break;
  }
}


/**
  * @brief  Отключение светодиодной индикации аварии обмена
  * @param
  * @retval
  */
void errIndGlowOff(void)
{
#warning реализовать согласно настройкам периферии
  //TIM_SetCounter(TIM17, 0);           //Сброс таймера аварии обмена
  //glowErrState = RESET;               //Сброс индикации аварии обмена
  //GPIO_ResetBits(GPIOC, GPIO_Pin_15); //Сброс индикации аварии обмена
}

void UartSlaveRx(void)
{

   __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
   
   memset(&rxUartBuf[0], 0, sizeof(rxUartBuf));
   
   HAL_UART_Receive_DMA(&huart1, (uint8_t*)rxUartBuf, sizeof(rxUartBuf));  
   
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
