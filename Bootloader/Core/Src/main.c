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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tinycrypt/sha256.h"
#include "tinycrypt/ecc_dsa.h"
#include "tinycrypt/ecc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* USER CODE BEGIN PV */
unsigned char public_key_der[] = {
  0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x02,
  0x01, 0x06, 0x08, 0x2a, 0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
  0x42, 0x00, 0x04, 0xcc, 0x55, 0xbe, 0x11, 0x0e, 0xf6, 0xb6, 0x62, 0x8f,
  0x9f, 0x0b, 0x80, 0x0d, 0x2b, 0xd3, 0xe3, 0xcc, 0xfc, 0x7f, 0x84, 0x9c,
  0x1e, 0x26, 0x6a, 0xca, 0x44, 0x2f, 0x68, 0x8c, 0xf4, 0xbb, 0x17, 0xc3,
  0x0e, 0x9c, 0x24, 0x62, 0x9c, 0x97, 0x24, 0x6b, 0x3b, 0x92, 0xa3, 0x07,
  0xc7, 0xd0, 0x63, 0x46, 0x08, 0x3a, 0x0a, 0x0b, 0xda, 0x92, 0xba, 0x3c,
  0x63, 0x78, 0x68, 0x62, 0x02, 0xd4, 0xc1
};
unsigned int public_key_der_len = 91;
/* USER CODE END PV */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BLOCK_SIZE          ( 1024 )                  //1KB
#define ETX_APP_START_ADDRESS   0x08004400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAJOR 1
#define MINOR 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t BL_Version[2] = { MAJOR, MINOR };
uint16_t application_size = 0;
uint16_t application_write_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void goto_application( void );
static void Firmware_Update( void );
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

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Bootloader v%d:%d Started!!!\n", BL_Version[0], BL_Version[1]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Firmware_Update();

  // Jump to application
  goto_application();
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{


  /* USER CODE BEGIN USART1_Init 1 */
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
  /* USER CODE END USART1_Init 1 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 1 */

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END USART3_Init 1 */
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

  return ch;
}


static int UART_Write_Loop( void )
{
  char tx = 'g';
  char rx = '0';
  HAL_StatusTypeDef ex;
  int ret = 0;
  int count = 0;

  while(1)
  {
    //Toggle GPIO
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    HAL_UART_Transmit(&huart3, (uint8_t *)&tx, 1, HAL_MAX_DELAY);

    ex = HAL_UART_Receive(&huart3, (uint8_t *)&rx, 1, 10);

    if( ( ex == HAL_OK ) && ( rx == 'r' ) )
    {
      //received data
      printf("Firmware Update Started\r\n");
      ret = 1;
      break;
    }

    if( count == 100 )
    {
      //received nothing
      printf("No Data Received for Firmware Update\r\n");
      break;
    }
    count++;
    HAL_Delay(20);              //20ms delay
  }

  return ret;
}

/**
  * @brief Write data to the Application's actual flash location.
  * @param data data to be written
  * @param data_len data length
  * @is_first_block true - if this is first block, false - not first block
  * @retval HAL_StatusTypeDef
  */
static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data,
                                        uint16_t data_len, bool is_first_block )
{
  HAL_StatusTypeDef ret;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //No need to erase every time. Erase only the first time.
    if( is_first_block )
    {
      printf("Erasing the Flash memory...\r\n");
      //Erase the Flash
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError;

      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.PageAddress   = ETX_APP_START_ADDRESS;
      EraseInitStruct.NbPages       = 47;                     //47 Pages

      ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
      if( ret != HAL_OK )
      {
        break;
      }
      application_write_idx = 0;
    }

    for(int i = 0; i < data_len/2; i++)
    {
      uint16_t halfword_data = data[i * 2] | (data[i * 2 + 1] << 8);
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD,
                               (ETX_APP_START_ADDRESS + application_write_idx ),
                               halfword_data
                             );
      if( ret == HAL_OK )
      {
        //update the data count
        application_write_idx += 2;
      }
      else
      {
        printf("Flash Write Error...HALT!!!\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while( false );

  return ret;
}


/**
  * @brief Check for Firmware Update and update the Firmware
  * @retval None
  */



static void Firmware_Update(void)
{
  uint8_t xx,yy;
  uint8_t x = 'x';
  uint8_t y = 'y';
  HAL_StatusTypeDef ex = HAL_OK;
  uint16_t current_app_size = 0;
  uint16_t i = 0;

  uint8_t block[MAX_BLOCK_SIZE] = { 0 };

  do
  {
    if( UART_Write_Loop() != 0 )
    {
      //Sender is ready. Receive the Firmware Size

      // Ask yy
      HAL_UART_Transmit(&huart3, &y, 1, HAL_MAX_DELAY);
      ex = HAL_UART_Receive(&huart3, &yy, 1, 5000);
      if( ex != HAL_OK )
      {
        printf("Get application Size error (yy)...HALT!!!\r\n");
        break;
      }

      // Ask xx
      HAL_UART_Transmit(&huart3, &x, 1, HAL_MAX_DELAY);
      ex = HAL_UART_Receive(&huart3, &xx, 1, 5000);
      if( ex != HAL_OK )
      {
        printf("Get application Size error(XX)...HALT!!!\r\n");
        break;
      }

      application_size = yy | (xx << 8);
      printf("Application Size = %d bytes\r\n", application_size);

      while(1)
      {
        if( ( i == MAX_BLOCK_SIZE ) || ( current_app_size >= application_size) )
        {
          printf("Received Block[%d]\r\n", current_app_size/MAX_BLOCK_SIZE);

          //write to flash
          ex = write_data_to_flash_app(block, MAX_BLOCK_SIZE, (current_app_size <= MAX_BLOCK_SIZE) );

          if( ex != HAL_OK )
          {
            break;
          }

          //clear the memory
          memset(block, 0,MAX_BLOCK_SIZE);
          i = 0;
        }

        if( current_app_size >= application_size)
        {
          //received all data. exit
          ex = HAL_OK;
          break;
        }

        // Ask yy
        HAL_UART_Transmit(&huart3, &y, 1, HAL_MAX_DELAY);
        ex = HAL_UART_Receive(&huart3, &yy, 1, 5000);
        if( ex != HAL_OK )
        {
          printf("Get application data[index:%d] error (yy)...HALT!!!\r\n", i);
          break;
        }

        // Ask xx
        HAL_UART_Transmit(&huart3, &x, 1, HAL_MAX_DELAY);
        ex = HAL_UART_Receive(&huart3, &xx, 1, 5000);
        if( ex != HAL_OK )
        {
          printf("Get application data[index:%d] error(XX)...HALT!!!\r\n", i);
          break;
        }

        //--- Save xxyy in block[i]
        block[i++] = yy;
        block[i++] = xx;
        current_app_size += 2;
      }
    }
  }
  while( false );

  if( ex != HAL_OK )
  {
    while(1);
  }
}

/* USER CODE BEGIN 4 */
bool verify_firmware_signature(const uint8_t *data, size_t data_len, const uint8_t *signature, size_t sig_len)
{
    struct tc_sha256_state_struct sha256_ctx;
    uint8_t hash[TC_SHA256_DIGEST_SIZE];
    uint8_t pub_key[64]; // Raw public key (x and y coordinates)

    // Convert DER encoded public key to raw format (x and y coordinates)
    memcpy(pub_key, &public_key_der[26], 32); // x coordinate
    memcpy(&pub_key[32], &public_key_der[58], 32); // y coordinate

    // Initialize the SHA-256 context
    tc_sha256_init(&sha256_ctx);

    // Compute the SHA-256 hash of the firmware
    tc_sha256_update(&sha256_ctx, data, data_len);
    tc_sha256_final(hash, &sha256_ctx);

    // Verify the ECDSA signature
    int verified = uECC_verify(pub_key, hash, TC_SHA256_DIGEST_SIZE, signature, uECC_secp256r1());

    if (verified == 0) {
        printf("Signature verification failed\n");
        return false;
    }

    return true;
}

static void goto_application(void)
{
    printf("Gonna Jump to Application...\n");

    uint8_t firmware[application_size];
    // Load firmware into the buffer
    memcpy(firmware, (void *)ETX_APP_START_ADDRESS, application_size);

    uint8_t signature[64]; // Example size for ECDSA P-256
    // Load signature from storage
    memcpy(signature, (void *)(ETX_APP_START_ADDRESS + application_size), sizeof(signature));

    if (!verify_firmware_signature(firmware, application_size, signature, sizeof(signature)))
    {
        printf("Invalid firmware signature... HALT!!!\r\n");
        while (1);
    }

    void (*app_reset_handler)(void) = (void *)(*((volatile uint32_t *)(ETX_APP_START_ADDRESS + 4U)));

    if (app_reset_handler == (void *)0xFFFFFFFF)
    {
        printf("Invalid Application... HALT!!!\r\n");
        while (1);
    }

    __set_MSP(*(volatile uint32_t *)ETX_APP_START_ADDRESS);

    // Turn OFF the LED to tell the user that Bootloader is not running
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    app_reset_handler(); // call the app reset handler
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
