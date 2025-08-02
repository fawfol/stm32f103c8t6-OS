/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Pin definitions for Blue Pill */
#define LED_PIN    GPIO_PIN_13
#define LED_PORT   GPIOC

#define SD_CS_PIN  GPIO_PIN_0
#define SD_CS_PORT GPIOB

/* SD Commands */
#define CMD0_GO_IDLE_STATE      0x40
#define CMD8_SEND_IF_COND       0x48
#define CMD55_APP_CMD           0x77
#define ACMD41_APP_OP_COND      0x69

/* Responses */
#define R1_IDLE_STATE           0x01
#define R1_OK                   0x00

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void Error_Handler(void);

/* Helper functions */
static void SD_CS_HIGH(void) { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET); }
static void SD_CS_LOW(void)  { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET); }

static uint8_t SPI_TransmitReceive(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response;

    SPI_TransmitReceive(cmd);
    SPI_TransmitReceive((arg >> 24) & 0xFF);
    SPI_TransmitReceive((arg >> 16) & 0xFF);
    SPI_TransmitReceive((arg >> 8) & 0xFF);
    SPI_TransmitReceive(arg & 0xFF);
    SPI_TransmitReceive(crc);

    uint16_t timeout = 0xFFFF;
    do {
        response = SPI_TransmitReceive(0xFF);
        timeout--;
    } while (response == 0xFF && timeout);

    return response;
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    /* Blink 3 times to confirm LED works */
    for(int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // ON
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   // OFF
        HAL_Delay(200);
    }

    /* SD Card Init */
    uint8_t sd_initialized = 0;
    uint8_t response;

    /* 80 dummy clocks with CS high */
    SD_CS_HIGH();
    for (int i = 0; i < 10; i++) SPI_TransmitReceive(0xFF);

    /* CMD0: Go to Idle */
    SD_CS_LOW();
    response = SD_SendCmd(CMD0_GO_IDLE_STATE, 0x00000000, 0x95);
    SD_CS_HIGH();

    if (response == R1_IDLE_STATE) {
        /* CMD8: Check voltage */
        SD_CS_LOW();
        response = SD_SendCmd(CMD8_SEND_IF_COND, 0x000001AA, 0x87);
        for(int i = 0; i < 4; i++) SPI_TransmitReceive(0xFF); // discard R7
        SD_CS_HIGH();

        if (response == R1_IDLE_STATE) {
            /* CMD55 + ACMD41 loop */
            uint16_t timeout = 5000;
            do {
                SD_CS_LOW();
                SD_SendCmd(CMD55_APP_CMD, 0x00000000, 0x01);
                SD_CS_HIGH();

                SD_CS_LOW();
                response = SD_SendCmd(ACMD41_APP_OP_COND, 0x40000000, 0x01);
                SD_CS_HIGH();
                HAL_Delay(1);
                timeout--;
            } while (response != R1_OK && timeout);

            if (response == R1_OK) sd_initialized = 1;
        }
    }

    /* Blink result */
    if (sd_initialized) {
        // Fast blink 10 times
        for(int i=0; i<10; i++) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(50);
        }
    } else {
        // Slow blink forever
        while (1) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(500);
        }
    }

    while (1) { }
}

/* System Clock */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

/* GPIO Init */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PC13 LED */
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* PB0 SD CS */
    GPIO_InitStruct.Pin = SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_CS_PORT, &GPIO_InitStruct);

    /* Default states */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   // LED off
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET); // CS high
}

/* SPI Init */
static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
