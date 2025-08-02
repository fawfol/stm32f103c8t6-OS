/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN_UP     GPIO_PIN_0
#define BTN_DOWN   GPIO_PIN_1
#define BTN_LEFT   GPIO_PIN_2
#define BTN_RIGHT  GPIO_PIN_3
#define BTN_OK     GPIO_PIN_4
#define BTN_PORT   GPIOA

// PC13 is the onboard LED for the Bluepill board
#define LED_PIN    GPIO_PIN_13
#define LED_PORT   GPIOC

// SD Card Chip Select pin (moved to PB0)
#define SD_CS_PIN  GPIO_PIN_0
#define SD_CS_PORT GPIOB

// SD Card commands and responses
#define CMD0_GO_IDLE_STATE      0x40 // CMD0 (0x00) | 0x40 for command token
#define CMD0_ARG                0x00000000
#define CMD0_CRC                0x94 // Valid CRC for CMD0_ARG
#define R1_IDLE_STATE           0x01

#define CMD8_SEND_IF_COND       0x48 // CMD8 (0x08) | 0x40
#define CMD8_ARG_VHS_27_36V     0x000001AA // Voltage supply + check pattern
#define CMD8_CRC                0x87 // Valid CRC for CMD8_ARG
#define R1_IN_IDLE_STATE        0x01 // Expected R1 for CMD8 in idle state

#define ACMD41_APP_OP_COND      0x69 // ACMD41 (0x29) | 0x40
#define ACMD41_ARG_HCS          0x40000000 // Host Capacity Support bit (HCS)
#define R1_OK                   0x00 // Expected R1 after initialization

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1; // Handle for SPI1 peripheral

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
// Helper functions for SD card communication
static void SD_CS_HIGH(void);
static void SD_CS_LOW(void);
static uint8_t SPI_TransmitReceive(uint8_t data);
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc);
static uint8_t SD_ReadR1Response(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Functions to control SD card Chip Select
static void SD_CS_HIGH(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static void SD_CS_LOW(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

// SPI Transmit/Receive single byte
static uint8_t SPI_TransmitReceive(uint8_t data) {
    uint8_t rx_data;
    // Transmit `data` and receive into `rx_data`. Timeout of 100ms
    if (HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100) != HAL_OK) {
        // Error handling if SPI communication fails
        Error_Handler();
    }
    return rx_data;
}

// Send SD Card Command
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response;
    // Transmit command byte
    SPI_TransmitReceive(cmd);

    // Transmit argument bytes (MSB first)
    SPI_TransmitReceive((arg >> 24) & 0xFF);
    SPI_TransmitReceive((arg >> 16) & 0xFF);
    SPI_TransmitReceive((arg >> 8) & 0xFF);
    SPI_TransmitReceive(arg & 0xFF);

    // Transmit CRC byte
    SPI_TransmitReceive(crc);

    // Wait for response (R1 response is a single byte, 0x01 in idle state)
    // Loop until a response byte is received or timeout
    uint16_t timeout = 0xFFF; // Arbitrary timeout value
    while ((response = SPI_TransmitReceive(0xFF)) == 0xFF && timeout > 0) {
        timeout--;
    }
    return response;
}

// Read R1 Response from SD Card
// This is typically called after sending a command.
// We keep reading 0xFF until we get a non-0xFF response (R1 response)
static uint8_t SD_ReadR1Response(void) {
    uint8_t response;
    uint16_t timeout = 0xFFF; // Max 8 clock cycles usually
    do {
        response = SPI_TransmitReceive(0xFF); // Send dummy byte to receive data
        timeout--;
    } while ((response == 0xFF) && (timeout > 0)); // Keep reading until non-0xFF or timeout
    return response;
}

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
  MX_SPI1_Init(); // Initialize SPI1 for SD card
  /* USER CODE BEGIN 2 */

  // --- SD Card Initialization Test ---
  uint8_t sd_status_r1;
  uint8_t sd_status_r7_bytes[4];
  uint8_t i;
  uint8_t sd_initialized = 0; // Flag to indicate successful SD card init

  // 1. Send 74+ clock cycles with CS high (inactive)
  SD_CS_HIGH(); // Ensure CS is high
  for (i = 0; i < 10; i++) { // Send 10 bytes (80 clock cycles) of 0xFF
      SPI_TransmitReceive(0xFF);
  }

  // 2. Send CMD0 (GO_IDLE_STATE) with CS low
  SD_CS_LOW(); // Activate CS
  sd_status_r1 = SD_SendCmd(CMD0_GO_IDLE_STATE, CMD0_ARG, CMD0_CRC);
  SD_CS_HIGH(); // Deactivate CS

  if (sd_status_r1 == R1_IDLE_STATE) { // Expected R1 response: 0x01
      // CMD0 successful, SD card is in idle state
      // Now, try CMD8 to check for SDHC/SDXC and voltage supply
      SD_CS_LOW(); // Activate CS
      sd_status_r1 = SD_SendCmd(CMD8_SEND_IF_COND, CMD8_ARG_VHS_27_36V, CMD8_CRC);
      if (sd_status_r1 == R1_IN_IDLE_STATE) { // Expected R1: 0x01
          // Read the 4-byte R7 response for CMD8
          for (i = 0; i < 4; i++) {
              sd_status_r7_bytes[i] = SPI_TransmitReceive(0xFF);
          }
          SD_CS_HIGH(); // Deactivate CS

          // Check if the voltage accepted and check pattern match
          if (sd_status_r7_bytes[2] == 0x01 && sd_status_r7_bytes[3] == 0xAA) {
              // Voltage accepted and check pattern matches
              // SD card is SDHC/SDXC or Standard Capacity SDv2
              // Now proceed with ACMD41 to initialize
              uint16_t acmd41_timeout = 5000; // Longer timeout for initialization
              do {
                  SD_CS_LOW(); // Activate CS
                  // Send CMD55 (APP_CMD) first, required before ACMD41
                  SD_SendCmd(0x77, 0x00000000, 0x01); // CMD55 (0x37) | 0x40 (dummy CRC)
                  SD_CS_HIGH(); // Deactivate CS

                  // Check R1 response for CMD55
                  SD_CS_LOW(); // Activate CS
                  sd_status_r1 = SD_SendCmd(ACMD41_APP_OP_COND, ACMD41_ARG_HCS, 0x01); // ACMD41 (0x29) | 0x40 (dummy CRC)
                  SD_CS_HIGH(); // Deactivate CS

                  acmd41_timeout--;
              } while (sd_status_r1 != R1_OK && acmd41_timeout > 0); // Loop until R1 is 0x00 (initialized)

              if (sd_status_r1 == R1_OK) {
                  sd_initialized = 1; // SD Card successfully initialized
              }
          }
      }
  }

  // --- LED Feedback for SD Card Status ---
  if (sd_initialized) {
      // SD card detected and initialized successfully!
      // PC13 LED (active-low) will blink rapidly to indicate success
      for(i=0; i<10; i++) {
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // ON
          HAL_Delay(50);
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   // OFF
          HAL_Delay(50);
      }
  } else {
      // SD card initialization failed.
      // PC13 LED (active-low) will stay ON (LOW) to indicate failure
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // Stay ON (LOW)
  }
  // --- End SD Card Initialization Test ---


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read the state of all buttons. If any button is pressed, turn on the LED.
	  // If all buttons are released, turn off the LED.
	  if (HAL_GPIO_ReadPin(BTN_PORT, BTN_UP) == GPIO_PIN_RESET ||
	      HAL_GPIO_ReadPin(BTN_PORT, BTN_DOWN) == GPIO_PIN_RESET ||
	      HAL_GPIO_ReadPin(BTN_PORT, BTN_LEFT) == GPIO_PIN_RESET ||
	      HAL_GPIO_ReadPin(BTN_PORT, BTN_RIGHT) == GPIO_PIN_RESET ||
	      HAL_GPIO_ReadPin(BTN_PORT, BTN_OK) == GPIO_PIN_RESET) { // All buttons now on BTN_PORT

	      // The PC13 LED (active-low) responds to buttons. If SD init failed,
	      // this will override the failure indication once a button is pressed.
	      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // ON
	  } else {
	      // If no button is pressed, turn the LED OFF.
	      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // OFF
	  }

	  // A small delay to prevent rapid polling issues, but not so long as to feel unresponsive.
	  HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // HSI (8MHz) is divided by 2
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // Then multiplied by 9 (4MHz * 9 = 36MHz)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // System clock = PLL output (36MHz)
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) // Correct flash latency for 36MHz
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER; // Microcontroller is the master
  hspi1.Init.Direction = SPI_DIRECTION_2LINES; // Full-duplex communication
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; // 8-bit data size for SD card commands/data
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE; // CPHA = 1 (often works, try SPI_PHASE_2EDGE if issues persist)
                                          // Typically CPOL=0/CPHA=0 or CPOL=1/CPHA=1 for SD cards, but CPOL=0/CPHA=1 observed to work with some
  hspi1.Init.NSS = SPI_NSS_SOFT; // Software slave select (we control CS manually on PA4)
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; // Initial low speed for SD card (PCLK2=36MHz / 256 = 140.625kHz)
                                                             // Can be increased later (e.g., SPI_BAUDRATEPRESCALER_4) for faster transfer after init
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; // Most Significant Bit first
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE; // Disable TI mode
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // CRC calculation disabled for now
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enabled for PC13 (LED)
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enabled for all buttons and SPI1 pins
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Enabled for PB0 (SD_CS)

  // Enable clock for Alternate Function I/O (AFIO)
  __HAL_RCC_AFIO_CLK_ENABLE();
  // __HAL_AFIO_REMAP_SPI1_DISABLE(); // SPI1 uses default pins PA4-PA7 by default. No remapping needed for this config.

  /*Configure GPIO pin Output Level (PC13 is active low) */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // Set OFF initially (HIGH for active-low LED)

  /*Configure GPIO pins : PC13 (Onboard LED) */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);


  /*Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 (All Buttons) */
  GPIO_InitStruct.Pin = BTN_UP|BTN_DOWN|BTN_LEFT|BTN_RIGHT|BTN_OK; // All buttons on GPIOA
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Buttons need pull-up resistors
  HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct); // Initialize GPIOA for all buttons


  /*Configure SPI1 GPIO pins : PA5 (SCK), PA6 (MISO), PA7 (MOSI) - Alternate Function */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate Function Push-Pull for SPI signals
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High speed for SPI
  GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up/down needed for AF pins
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Initialize GPIOA for SPI pins


  /*Configure GPIO pin : PB0 (SD Card Chip Select - Manual Control) */
  GPIO_InitStruct.Pin = SD_CS_PIN; // PB0
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // General purpose output for CS
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Keep CS high by default (inactive)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_PORT, &GPIO_InitStruct); // Initialize GPIOB for SD_CS pin

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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
