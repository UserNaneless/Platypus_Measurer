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
#include "stm32f1xx_hal_spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define wait HAL_Delay
#define ACC_RANGE 0x41
#define ACC_CONF 0x40
#define ACC_SELF_TEST 0x6D
#define ACC_POWER_CTL 0x7D
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

typedef struct LED_Color_Data {
        GPIO_TypeDef *GPIO_Port;
        uint16_t GPIO_Pin;
} LED_Color_Data;

typedef struct LED_RGB_Data {
        LED_Color_Data r;
        LED_Color_Data g;
        LED_Color_Data b;
} LED_RGB_Data;

enum Color {
    WHITE = 0b111,
    BLUE = 0b001,
    GREEN = 0b010,
    RED = 0b100,
    CYAN = 0b011,
    MAGENTA = 0b101,
    YELLOW = 0b110,
    OFF = 0b000
};

class LED {
    private:
        LED_RGB_Data led;
        Color ledRGB = Color::OFF;

        void setPin(GPIO_TypeDef *port, uint16_t pin, bool state) {
            if (state)
                HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        }

    public:
        LED(LED_RGB_Data &led) : led(led) {}

        void setColor(Color color) {
            ledRGB = color;
            setPin(led.r.GPIO_Port, led.r.GPIO_Pin, (color >> 2) & 1);
            setPin(led.g.GPIO_Port, led.g.GPIO_Pin, (color >> 1) & 1);
            setPin(led.b.GPIO_Port, led.b.GPIO_Pin, color & 1);
        }

        void Off() {
            setColor(Color::OFF);
        }

        void OK() {
            setColor(ledRGB);
            wait(300);
            Off();
            wait(300);
            setColor(ledRGB);
            wait(300);
            Off();
            wait(300);
            setColor(ledRGB);
            wait(300);
            Off();
        }

        void OK(Color color) {
            setColor(color);
            wait(300);
            Off();
            wait(300);
            setColor(color);
            wait(300);
            Off();
            wait(300);
            setColor(color);
            wait(300);
            Off();
        }
};

void accelerometer() {
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_SET);
};

void gyroscope() {
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_RESET);
}

void stop() {
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_SET);
}

void writeToReg(uint8_t reg, uint8_t *value, size_t val_size) {
    HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
    HAL_SPI_Transmit(&hspi1, value, val_size, 1000);
}

void acceleromeretOn() {
    writeToReg(ACC_POWER_CTL, (uint8_t *)0x04, 1);
}

void accelerometerBIT() {
    //+-24g
    writeToReg(ACC_RANGE, (uint8_t *)0x03, 1);

    // 1.6khz
    writeToReg(ACC_CONF, (uint8_t *)0xA7, 1);

    wait(2);

    // self test polarity +
    writeToReg(ACC_SELF_TEST, (uint8_t *)0x0D, 1);

    wait(50);

    uint8_t tx = 0x12;
    uint8_t positive[6] = { 0 };
    uint8_t pRange = 0;
    uint8_t range = ACC_RANGE;
    uint8_t negative[6] = { 0 };
    uint8_t nRange = 0;

    HAL_SPI_TransmitReceive(&hspi1, &tx, positive, 6, 1000);

    HAL_SPI_TransmitReceive(&hspi1, &range, &pRange, 1, 1000);

    // self test polarity -
    writeToReg(ACC_SELF_TEST, (uint8_t *)0x09, 1);

    wait(50);

    HAL_SPI_TransmitReceive(&hspi1, &tx, negative, 6, 1000);

    HAL_SPI_TransmitReceive(&hspi1, &range, &nRange, 1, 1000);

    // disable self test
    writeToReg(ACC_SELF_TEST, (uint8_t *)0x00, 1);

    int pAccel[3] = {
        positive[1] * 256 + positive[0] / 32768 * 1000 * (1 << (pRange + 1)),
        positive[3] * 256 + positive[2] / 32768 * 1000 * (1 << (pRange + 1)),
        positive[5] * 256 + positive[4] / 32768 * 1000 * (1 << (pRange + 1)),
    };

    int nAccel[3] = {
        negative[1] * 256 + negative[0] / 32768 * 1000 * (1 << (nRange + 1)),
        negative[3] * 256 + negative[2] / 32768 * 1000 * (1 << (nRange + 1)),
        negative[5] * 256 + negative[4] / 32768 * 1000 * (1 << (nRange + 1)),
    };

    wait(50);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

    acceleromeretOn();

    LED_RGB_Data Led1_RGB = {
        .r = { .GPIO_Port = LED1_RGB_RED_GPIO_Port, .GPIO_Pin = LED1_RGB_RED_Pin },
        .g = { .GPIO_Port = LED1_RGB_GREEN_GPIO_Port, .GPIO_Pin = LED1_RGB_GREEN_Pin },
        .b = { .GPIO_Port = LED1_RGB_BLUE_GPIO_Port, .GPIO_Pin = LED1_RGB_BLUE_Pin }
    };

    LED led1(Led1_RGB);

    uint8_t tx = 0x12 | 0x80;
    uint8_t rx[6];
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        accelerometer();
        accelerometerBIT();
        stop();

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED1_RGB_RED_Pin | LED1_RGB_GREEN_Pin | LED1_RGB_BLUE_Pin | B_ACCELEROMETER_Pin | B_GYROSCOPE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LED1_RGB_RED_Pin LED1_RGB_GREEN_Pin LED1_RGB_BLUE_Pin B_ACCELEROMETER_Pin
                             B_GYROSCOPE_Pin */
    GPIO_InitStruct.Pin = LED1_RGB_RED_Pin | LED1_RGB_GREEN_Pin | LED1_RGB_BLUE_Pin | B_ACCELEROMETER_Pin | B_GYROSCOPE_Pin;
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
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
