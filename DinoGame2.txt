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
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define GROUND_Y 50

// Button pins
#define BTN_OK     GPIO_PIN_0
#define BTN_UP     GPIO_PIN_1
#define BTN_DOWN   GPIO_PIN_2
#define BTN_LEFT   GPIO_PIN_3
#define BTN_RIGHT  GPIO_PIN_4
#define BTN_PORT   GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Dino game variables
int dinoY = GROUND_Y - 10;
int jump = 0;
int velocity = 0;

// Cactus variables
int cactusX = SCREEN_WIDTH;
int cactusWidth = 4;
int cactusHeight = 9;
int cactusSpeed = 3;

// Game state
int gameRunning = 0;
int score = 0;
// Button previous states for edge detection
uint8_t prevBtnUp = 1;  // 1 = not pressed (pull-up)
uint8_t prevBtnOk = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void drawGame(void);
void handleInput(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void drawGame(void)
{
    char score_str[16]; // Buffer to hold the score string
    ssd1306_Fill(Black);

    // Draw ground
    for (int x = 0; x < SCREEN_WIDTH; x++) {
        ssd1306_DrawPixel(x, GROUND_Y, White);
    }

    // Draw dino (8x8 block)
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            ssd1306_DrawPixel(10 + x, dinoY + y, White);
        }
    }

    // Draw cactus
    ssd1306_DrawRectangle(cactusX, GROUND_Y - cactusHeight, cactusX + cactusWidth, GROUND_Y, White);

    // Display Score
    sprintf(score_str, "Score: %d", score); // Convert score to string
    ssd1306_SetCursor(5, 5); // Top-left corner for score
    ssd1306_WriteString(score_str, Font_6x8, White); // Use Font_6x8 for smaller text

    ssd1306_UpdateScreen();
}


void handleInput(void)
{
    uint8_t btnUp = HAL_GPIO_ReadPin(BTN_PORT, BTN_UP);
    uint8_t btnOk = HAL_GPIO_ReadPin(BTN_PORT, BTN_OK);

    // Detect OK button press (falling edge)
    if(!gameRunning && btnOk == GPIO_PIN_RESET && prevBtnOk == GPIO_PIN_SET) {
        gameRunning = 1;
        cactusX = SCREEN_WIDTH;
        dinoY = GROUND_Y - 8;
        jump = 0;
        velocity = 0;
        score = 0;          // Reset score
        cactusSpeed = 3;    // Reset speed to initial value
    }

    // Detect UP button press (falling edge)
    if(gameRunning && btnUp == GPIO_PIN_RESET && prevBtnUp == GPIO_PIN_SET && jump == 0) {
        jump = 1;
        velocity = -6;
    }

    // Update previous states
    prevBtnUp = btnUp;
    prevBtnOk = btnOk;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      handleInput();

      if(gameRunning) {
          // Jump physics
          if(jump) {
              dinoY += velocity;
              velocity++;
              if(dinoY >= GROUND_Y - 8) {
                  dinoY = GROUND_Y - 8;
                  jump = 0;
              }
          }

          // Move cactus
          cactusX -= cactusSpeed;
          if(cactusX < -cactusWidth) {
              cactusX = SCREEN_WIDTH; // reset cactus
              score++; // Increment score when a cactus is passed!

              // Increase speed every 5 points
              if (score > 0 && score % 2 == 0) { // Make sure score is greater than 0 to avoid increasing speed at game start
                  cactusSpeed++;
              }
          }

          // Collision detection
          if(10 + 8 > cactusX && 10 < cactusX + cactusWidth &&
             dinoY + 8 > GROUND_Y - cactusHeight) {
              gameRunning = 0;
              ssd1306_Fill(Black);
              ssd1306_SetCursor(20, 25);
              ssd1306_WriteString("GAME OVER!", Font_7x10, White);
              // Optionally display final score on game over screen
              char final_score_str[16];
              sprintf(final_score_str, "Score: %d", score);
              ssd1306_SetCursor(30, 35); // Adjust position as needed
              ssd1306_WriteString(final_score_str, Font_6x8, White);
              ssd1306_UpdateScreen();
              HAL_Delay(2000); // Give player more time to see score
          }

          drawGame();
      } else {
          // Show start screen
          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 25);
          ssd1306_WriteString("Press OK to Start", Font_7x10, White);
          ssd1306_UpdateScreen();
      }

      HAL_Delay(50);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
