/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "platform.h"
#include <string.h>

//Инструкции для обмена с VPC3
#define READ_BYTE_INSTRUCTION   0x13
#define READ_ARRAY_INSTRUCTION  0x03
#define WRITE_BYTE_INSTRUCTION  0x12
#define WRITE_ARRAY_INSTRUCTION 0x02

#warning Определиться с длиной
#define MAX_DOUT_BUF 30
#define MAX_DIN_BUF 30

uint8_t tmpBuf[128] = {0};

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

#warning Определиться со временем
#define TIMEOUT_VAL 300

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/*---------------------------------------------------------------------------*/
/* function: DpAppl_SetResetVPC3Channel1                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Set VPC3+ reset.
 *
 * @attention The VPC3+ reset is high-active!
 *
 */
void DpAppl_SetResetVPC3Channel1( void )
{
   /** @todo Add your own code here! */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // 
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_ClrResetVPC3Channel1                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Clear VPC3+ reset.
 *
 * @attention The VPC3+ reset is high-active!
 *
 */
void DpAppl_ClrResetVPC3Channel1( void )
{
   /** @todo Add your own code here! */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableInterruptVPC3Channel1                              */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable VPC3+ interrupt.
 */
void DpAppl_EnableInterruptVPC3Channel1( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableInterruptVPC3Channel1                             */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable VPC3+ interrupt.
 *
 */
void DpAppl_DisableInterruptVPC3Channel1( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableInterruptVPC3Sync                                  */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable VPC3+ isochronous interrupt.
 *
 * @attention Is only supported from VPC3+S!
 *
 */
void DpAppl_EnableInterruptVPC3Sync( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableInterruptVPC3Sync                                 */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable VPC3+ isochronous interrupt.
 *
 * @attention Is only supported from VPC3+S!
 *
 */
void DpAppl_DisableInterruptVPC3Sync( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableAllInterrupts                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable all microcontroller interrupts.
 *
 */
void DpAppl_EnableAllInterrupts( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableAllInterrupts                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable all microcontroller interrupts.
 *
 */
void DpAppl_DisableAllInterrupts( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: Vpc3Wait_1ms                                                    */
/*---------------------------------------------------------------------------*/
/**
 * @brief Wait some time.
 *
 */
void Vpc3Wait_1ms( void )
{
   /** @todo Add your own code here! */
}



/*---------------------------------------------------------------------------*/
/* function: Vpc3Write                                                       */
/*---------------------------------------------------------------------------*/
/**
 * @brief Write a byte to VPC3+.
 *
 * @attention This function is only necessary with VPC3+S in SPI- or IIC-Mode!
 *
 * @param[in]wAddress Address in VPC3+
 * @param[in]bData Data
 */
#if VPC3_SERIAL_MODE
void Vpc3Write( VPC3_ADR wAddress, uint8_t bData )
{
   /** @todo Add your own code here! */
  uint16_t SPI_dataLength;
  uint8_t WrBuf[4] = {0}; //Локальный буфер для отправки
  
  WrBuf[0] = WRITE_BYTE_INSTRUCTION; //Код операции
  WrBuf[1] = (wAddress >> 8) & 0x00FFU;     //Старший байт адреса
  WrBuf[2] = wAddress & 0x00FFU;            //Младший байт адреса
  WrBuf[3] = bData;
  
  SPI_dataLength = 4;                             //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Transmit(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High 
  
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: Vpc3Read                                                        */
/*---------------------------------------------------------------------------*/
/**
 * @brief Read one byte from VPC3+.
 *
 * @attention This function is only necessary with VPC3+S in SPI- or IIC-Mode!
 *
 * @param[in]wAddress Address in VPC3+
 * @return value of wAddress
 */
#if VPC3_SERIAL_MODE
uint8_t Vpc3Read( VPC3_ADR wAddress )
{
   /** @todo Add your own code here! */
  uint8_t WrBuf[4] = {0}; //Локальный буфер для отправки
  uint16_t SPI_dataLength;
  
  WrBuf[0] = READ_BYTE_INSTRUCTION; //Код операции
  WrBuf[1] = (wAddress >> 8) & 0x00FFU;           //Старший байт адреса
  WrBuf[2] = wAddress & 0x00FFU;                  //Младший байт адреса
  
  SPI_dataLength = 4;                             //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Receive(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High
  
  return WrBuf[3];
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: Vpc3MemSet                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Fill block of VPC3+ memory.
 *
 * @param[in]wAddress Address of the block of memory to fill.
 * @param[in]bValue  Value to be set.
 * @param[in]wLength Number of bytes to be set to the value.
 */
#if VPC3_SERIAL_MODE
void Vpc3MemSet( VPC3_ADR wAddress, uint8_t bValue, uint16_t wLength )
{
   /** @todo Add your own code here! */
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: Vpc3MemCmp                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Compare two blocks of VPC3+ memory.
 *
 * @param[in]pToVpc3Memory1 Pointer to block of memory.
 * @param[in]pToVpc3Memory2 Pointer to block of memory.
 * @param[in]wLength Number of bytes to compare.
 * @return 0 Indicates that the contents of both memory blocks are equal.
 * @return 1 Indicates that the contents of both memory blocks are not equal.
 */
#if VPC3_SERIAL_MODE
uint8_t Vpc3MemCmp( VPC3_UNSIGNED8_PTR pToVpc3Memory1, VPC3_UNSIGNED8_PTR pToVpc3Memory2, uint16_t wLength )
{
   /** @todo Add your own code here! */

uint8_t bRetValue;
uint16_t i;

   bRetValue = 0;
   for( i = 0; i < wLength; i++ )
   {
      if( Vpc3Read( (VPC3_ADR)pToVpc3Memory1++ ) != Vpc3Read( (VPC3_ADR)pToVpc3Memory2++ ) )
      {
         bRetValue = 1;
         break;
      }
   }

   return bRetValue;
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: CopyToVpc3                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Copy block of memory to VPC3+.
 *
 * @param[in]pToVpc3Memory Pointer to the destination array where the content is to be copied.
 * @param[in]pLocalMemory Pointer to the source of data to be copied.
 * @param[in]wLength Number of bytes to copy.
 */
#if VPC3_SERIAL_MODE
void CopyToVpc3( VPC3_UNSIGNED8_PTR pToVpc3Memory, MEM_UNSIGNED8_PTR pLocalMemory, uint16_t wLength )
{
   /** @todo Add your own code here! */
  uint16_t SPI_dataLength;
  uint8_t WrBuf[MAX_DOUT_BUF] = {0}; //Локальный буфер для отправки
  
  WrBuf[0] = WRITE_ARRAY_INSTRUCTION; //Код операции
  WrBuf[1] = ((uint16_t)pToVpc3Memory >> 8) & 0x00FFU;                  //Старший байт адреса
  WrBuf[2] = (uint16_t)pToVpc3Memory & 0x00FFU;                         //Младший байт адреса
  
  memcpy((uint8_t *)&WrBuf[3], (uint8_t *)pLocalMemory, wLength); //Копируем значения для передачи
  SPI_dataLength = 3 + wLength;                                   //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Transmit(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High 
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: CopyFromVpc3                                                    */
/*---------------------------------------------------------------------------*/
#if VPC3_SERIAL_MODE
/**
 * @brief Copy block of memory from VPC3+.
 *
 * @param[in]pLocalMemory Pointer to the destination array where the content is to be copied.
 * @param[in]pToVpc3Memory Pointer to the source of data to be copied.
 * @param[in]wLength Number of bytes to copy.
 */
void CopyFromVpc3( MEM_UNSIGNED8_PTR pLocalMemory, VPC3_UNSIGNED8_PTR pToVpc3Memory, uint16_t wLength )
{
   /** @todo Add your own code here! */
  uint8_t WrBuf[MAX_DOUT_BUF] = {0}; //Локальный буфер для отправки
  uint16_t SPI_dataLength;
   
  WrBuf[0] = READ_ARRAY_INSTRUCTION; //Код операции
  WrBuf[1] = ((uint16_t)pToVpc3Memory >> 8) & 0x00FFU;                  //Старший байт адреса
  WrBuf[2] = (uint16_t)pToVpc3Memory & 0x00FFU;                         //Младший байт адреса
  
  SPI_dataLength = 3 + wLength;                             //3 служебных байта + размер данных
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); //CS = Low
  HAL_SPI_Receive(&hspi1, &WrBuf[0], SPI_dataLength, TIMEOUT_VAL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  //CS = High
  
  memcpy((uint8_t *)pLocalMemory, (uint8_t *)&WrBuf[3], wLength); //Копируем считанные значения
  
}
#endif//#if VPC3_SERIAL_MODE



/*---------------------------------------------------------------------------*/
/* function: TestVpc3_01                                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Hardware test of VPC3+.
 * If you get problems with reading VPC3+, you should read first the status register address 5.
 * The default value is CFhex (VPC3+C) or EFhex (VPC3+S).
 * Check reset signal of VPC3+ (notice: reset is high active).
 *
 */
void TestVpc3_01( void )
{
   uint8_t bValue;

   DpAppl_SetResetVPC3Channel1();
   Vpc3Wait_1ms();
   DpAppl_ClrResetVPC3Channel1();

   while(1)
   {
      #if VPC3_SERIAL_MODE
         bValue = Vpc3Read(0x05);
      #else
         bValue = *((unsigned char *)0x28005); //address depends on hardware!!!!!
      #endif//#if VPC3_SERIAL_MODE
   }//while(1)
}



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  //UartSlaveRx(); //Запуск приема данных от CP24
  
  DpAppl_SetResetVPC3Channel1();

  //initialize VPC3+C/S
  DpAppl_ProfibusInit();

   //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
   memset(&tmpBuf[0], 0, sizeof(tmpBuf));
   
  while (1)
  {
      // call PROFIBUS
    //HAL_UART_Receive_DMA(&huart1, &tmpBuf[0], 6);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &tmpBuf[0], 6);
      DpAppl_ProfibusMain();
  }
}



/*---------------------------------------------------------------------------*/
/* interrupt: external interrupt 0                                           */
/*---------------------------------------------------------------------------*/
#ifdef DPV_ISR_PROCESSING
   #if( VPC3_SERIAL_MODE == 0 )

      void ex0_ISR (void) interrupt IRQ_INT0
      {
         VPC3_Isr();
      }//void ex0_ISR (void) interrupt IRQ_INT0

   #endif//#if( VPC3_SERIAL_MODE == 0 )
#endif//#ifdef DPV_ISR_PROCESSING


/**
  * @brief  Обработка прерывания от VPC3+S
  * @param  None
  * @retval None
  */
void VPC3_InterruptProcessing(void)
{
  return;
}



/*****************************************************************************/
/*  Copyright (C) profichip GmbH 2009. Confidential.                         */
/*****************************************************************************/


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

