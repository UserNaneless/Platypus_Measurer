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
#include "CAN.h"
#include "Informator.h"
#include "Led.h"
#include "LedBase.h"
#include "LedRgb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include <cstring>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define wait HAL_Delay
#define STATUS HAL_StatusTypeDef

#define ACCELEROMETER accelerometer();
#define ACC_CHIP_ID 0x00
#define ACC_RANGE 0x41
#define ACC_CONF 0x40
#define ACC_SELF_TEST 0x6D
#define ACC_POWER_CTL 0x7D
#define ACC_X_LSB 0x12
#define ACC_X_MSB 0x13
#define ACC_Y_LSB 0x14
#define ACC_Y_MSB 0x15
#define ACC_Z_LSB 0x16
#define ACC_Z_MSB 0x17
#define ACC_PWR_CONF 0x7C
#define ACC_SOFT_RESET 0x7E

#define GYROSCOPE gyroscope();
#define GYRO_CHIP_ID 0x00
#define GYRO_RANGE 0x0F
#define GYRO_BANDWIDTH 0x10
#define GYRO_LPM1 0x11
#define GYRO_SOFTRESET 0x14
#define GYRO_RATE_Z_MSB 0x07
#define GYRO_RATE_Z_LSB 0x06
#define GYRO_RATE_Y_MSB 0x05
#define GYRO_RATE_Y_LSB 0x04
#define GYRO_RATE_X_MSB 0x03
#define GYRO_RATE_X_LSB 0x02

#define ACCELEROMETER_FLAG 0xAA
#define GYROSCOPE_FLAG 0xBB
#define ERROR_FLAG 0xEE
#define READY_FLAG 0xFF

#define ACCELEROMETER_ON \
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_RESET);
#define ACCELEROMETER_OFF \
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_SET);

#define GYROSCOPE_ON \
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_RESET);
#define GYROSCOPE_OFF \
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_SET);

#define CLOSE_OTHER_CS                                  \
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); \
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

#define STOP stop();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

LED *can_led = nullptr;
LED_RGB *led = nullptr;
CAN *pcan = nullptr;
Informator *pInformator_can = nullptr;
Informator *pInformator_led = nullptr;

void Error_Stop() {
    uint8_t data[1] = { ERROR_FLAG };
    pcan->SendMessage(data, 1);
    led->setColor(Color::RED);
    Error_Handler();
}

void accelerometer() {
    ACCELEROMETER_ON;
    GYROSCOPE_OFF;
}

void gyroscope() {
    ACCELEROMETER_OFF;
    GYROSCOPE_ON;
}

void stop() {
    ACCELEROMETER_OFF;
    GYROSCOPE_OFF;
}

STATUS writeToReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg, value };
    tx[0] &= 0x7F;
    STATUS status = HAL_OK;
    status = HAL_SPI_Transmit(&hspi1, tx, 2, 1000);
    STOP;

    return status;
}

uint8_t readFromReg(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2];
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 1000);
    STOP;
    return rx[1];
}

STATUS readFromReg(uint8_t reg, uint8_t *data, size_t size) {
    STATUS status = HAL_OK;
    uint8_t tx[1 + size];
    tx[0] = (uint8_t)(reg | 0x80);
    for (size_t i = 1; i < size; i++) {
        tx[i] = 0;
    }
    uint8_t rx[1 + size];
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, size + 1, 1000);
    STOP;
    memcpy(data, rx + 1, size);

    return status;
}

STATUS writeToReg_Acc(uint8_t reg, uint8_t value) {
    ACCELEROMETER;
    return writeToReg(reg, value);
}

uint8_t readFromReg_Acc(uint8_t reg) {
    ACCELEROMETER;
    uint8_t val = readFromReg(reg);
    return val;
}

STATUS readFromReg_Acc(uint8_t reg, uint8_t *data, size_t size) {
    ACCELEROMETER;
    return readFromReg(reg, data, size);
}

void accelerometerOn() {
    STATUS status = HAL_OK;

    status = writeToReg_Acc(ACC_SOFT_RESET, 0xB6);

    wait(100);

    status = writeToReg_Acc(ACC_PWR_CONF, 0x00);
    status = writeToReg_Acc(ACC_POWER_CTL, 0x04);

    wait(100);

    if (status != HAL_OK)
        Error_Stop();
}

bool accelerometerBIT() {
    STATUS status = HAL_OK;
    //+-24g
    status = writeToReg_Acc(ACC_RANGE, 0x03);
    if (status != HAL_OK)
        Error_Stop();

    uint8_t range = readFromReg_Acc(ACC_RANGE);
    if (range != 0x03)
        Error_Stop();

    // 1.6khz
    status = writeToReg_Acc(ACC_CONF, 0xA7);
    if (status != HAL_OK)
        Error_Stop();

    wait(2);

    // self test polarity +
    status = writeToReg_Acc(ACC_SELF_TEST, 0x0D);
    if (status != HAL_OK)
        Error_Stop();

    wait(50);
    uint8_t positive[6] = { 0 };
    uint8_t pRange = 0;
    uint8_t negative[6] = { 0 };
    uint8_t nRange = 0;

    status = readFromReg_Acc(ACC_X_LSB, positive, 6);
    if (status != HAL_OK)
        Error_Stop();
    // HAL_SPI_TransmitReceive(&hspi1, &tx, positive, 6, 1000);

    pRange = readFromReg_Acc(ACC_RANGE);
    if (pRange != 0x03)
        Error_Stop();

    // self test polarity -
    status = writeToReg_Acc(ACC_SELF_TEST, 0x09);
    if (status != HAL_OK)
        Error_Stop();

    wait(50);

    status = readFromReg_Acc(ACC_X_LSB, negative, 6);
    if (status != HAL_OK)
        Error_Stop();
    // HAL_SPI_TransmitReceive(&hspi1, &tx, negative, 6, 1000);

    nRange = readFromReg_Acc(ACC_RANGE);
    if (nRange != 0x03)
        Error_Stop();

    // disable self test
    status = writeToReg_Acc(ACC_SELF_TEST, 0x00);
    if (status != HAL_OK)
        Error_Stop();

    int16_t pAccel_16[3] = {
        (int16_t)((positive[1] << 8) | positive[0]),
        (int16_t)((positive[3] << 8) | positive[2]),
        (int16_t)((positive[5] << 8) | positive[4]),
    };

    int16_t nAccel_16[3] = {
        (int16_t)((negative[1] << 8) | negative[0]),
        (int16_t)((negative[3] << 8) | negative[2]),
        (int16_t)((negative[5] << 8) | negative[4]),
    };

    float pAccel[3] = {
        (float)pAccel_16[0] / 32768.0f * 1000.0f * (1 << (pRange + 1)),
        (float)pAccel_16[1] / 32768.0f * 1000.0f * (1 << (pRange + 1)),
        (float)pAccel_16[2] / 32768.0f * 1000.0f * (1 << (pRange + 1)),
    };

    float nAccel[3] = {
        (float)nAccel_16[0] / 32768.0f * 1000.0f * (1 << (nRange + 1)),
        (float)nAccel_16[1] / 32768.0f * 1000.0f * (1 << (nRange + 1)),
        (float)nAccel_16[2] / 32768.0f * 1000.0f * (1 << (nRange + 1)),
    };

    bool res = true;

    if (pAccel[0] - nAccel[0] < 1000 || pAccel[1] - nAccel[1] < 1000 || pAccel[2] - nAccel[2] < 500) {
        res = false;
    }

    wait(50);

    led->Off();

    return res;
}

STATUS writeToReg_Gyro(uint8_t reg, uint8_t value) {
    GYROSCOPE;
    return writeToReg(reg, value);
}

uint8_t readFromReg_Gyro(uint8_t reg) {
    GYROSCOPE;
    uint8_t val = readFromReg(reg);
    return val;
}

STATUS readFromReg_Gyro(uint8_t reg, uint8_t *data, size_t size) {
    GYROSCOPE;
    return readFromReg(reg, data, size);
}

STATUS getGyroData(uint8_t *data) {
    return readFromReg_Gyro(GYRO_RATE_X_LSB, data, 6);
}

STATUS getAccData(uint8_t *data) {
    return readFromReg_Acc(ACC_X_LSB, data, 6);
}

bool gyroscopeTest() {
    uint8_t data[6] = {};
    uint8_t chipID = readFromReg_Gyro(GYRO_CHIP_ID);

    readFromReg_Gyro(GYRO_RATE_X_LSB, data, 6);

    float x = (int16_t)((data[1] << 8) | data[0]) / 16.384f;
    float y = (int16_t)((data[3] << 8) | data[2]) / 16.384f;
    float z = (int16_t)((data[5] << 8) | data[4]) / 16.384f;

    if (chipID != 0x0F || (x == 0 && y == 0 && z == 0))
        return false;

    return true;
}

bool accelerometerTest() {
    led->setColor(Color::YELLOW);
    uint8_t data[6] = {};
    uint8_t _ = readFromReg_Acc(ACC_CHIP_ID);
    uint8_t chipID = readFromReg_Acc(ACC_CHIP_ID);
    // uint8_t pConf = readFromReg_Acc(ACC_PWR_CONF);

    STATUS status = readFromReg_Acc(ACC_X_LSB, data, 6);

    uint8_t error = readFromReg_Acc(0x02);

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);

    uint8_t error_code = (error >> 2) & 0b111;
    uint8_t fatal_code = error & 0b1;

    // if (chipID != 0x1E || error_code != 0 || fatal_code != 0)
    if (chipID != 0x1E || status != HAL_OK)
        return false;

    led->setColor(Color::GREEN);
    wait(50);
    led->setColor(Color::OFF);
    return true;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool isPinSet(GPIO_TypeDef *port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin);
}

class Measurer {
    private:
        CAN *can;
        uint32_t clock = 0;
        uint32_t slice = 30;

        uint32_t lastSend = 0;

    public:
        Measurer() {
        }

        void init(CAN *can) {
            this->can = can;
        }

        uint32_t &getClock() {
            return clock;
        }

        uint8_t readAddr() {
            const bool addr[5] = {
                !isPinSet(ADDR_0_GPIO_Port, ADDR_0_Pin),
                !isPinSet(ADDR_1_GPIO_Port, ADDR_1_Pin),
                !isPinSet(ADDR_2_GPIO_Port, ADDR_2_Pin),
                !isPinSet(ADDR_3_GPIO_Port, ADDR_3_Pin),
                !isPinSet(ADDR_4_GPIO_Port, ADDR_4_Pin),
            };

            return (addr[0] << 0) | (addr[1] << 1) | (addr[2] << 2) | (addr[3] << 3) | (addr[4] << 4);
        }

        void sendData() {
            uint8_t accel[7] = { ACCELEROMETER_FLAG, 0 };
            uint8_t gyro[7] = { GYROSCOPE_FLAG, 0 };

            getAccData(accel + 1);
            getGyroData(gyro + 1);

            can->SendMessage(accel, 7);
            wait(10);
            can->SendMessage(gyro, 7);
            pushSuccess(pInformator_led);
        }

        void tick() {
            clock = HAL_GetTick();

            if (clock - lastSend >= slice) {
                pushInfo(pInformator_can);
                sendData();
                lastSend = clock;
            }
        }

        void selfTest() {
            if (!gyroscopeTest() || !accelerometerTest())
                Error_Stop();

            STATUS status = writeToReg_Acc(ACC_CONF, 0b10011000); // normal 100hz
            if (status != HAL_OK)
                Error_Stop();

            uint8_t data[1] = { READY_FLAG };
            can->SendMessage(data, 1);
        }
};

void tester() {
}

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
    MX_CAN_Init();
    /* USER CODE BEGIN 2 */

    CLOSE_OTHER_CS;

    LED_Color_Data led_data = {
        .GPIO_Port = LED_GPIO_Port,
        .GPIO_Pin = LED_Pin
    };

    LED_RGB_Data LED1_RGB_Data = {
        {
            .GPIO_Port = LED1_RGB_RED_GPIO_Port,
            .GPIO_Pin = LED1_RGB_RED_Pin,
        },
        {
            .GPIO_Port = LED1_RGB_GREEN_GPIO_Port,
            .GPIO_Pin = LED1_RGB_GREEN_Pin,
        },
        {
            .GPIO_Port = LED1_RGB_BLUE_GPIO_Port,
            .GPIO_Pin = LED1_RGB_BLUE_Pin,
        },
    };

    Measurer measurer;

    LED CAN_LED(led_data);
    can_led = &CAN_LED;

    LED_RGB Led1(LED1_RGB_Data);
    led = &Led1;
    led->setColor(Color::MAGENTA);

    Informator Informator_can(can_led);
    Informator Informator_led(led);

    pInformator_can = &Informator_can;
    pInformator_led = &Informator_led;

    STATUS status = HAL_OK;
    status = writeToReg_Gyro(GYRO_SOFTRESET, 0xB6);
    wait(100);
    status = writeToReg_Gyro(GYRO_RANGE, 0x01);
    status = writeToReg_Gyro(GYRO_BANDWIDTH, 0x02);
    status = writeToReg_Gyro(GYRO_LPM1, 0x00);

    if (status != HAL_OK)
        Error_Stop();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    CAN_Init can_data = {
        .hcan = &hcan,
        .id = measurer.readAddr(),
        .led = led
    };

    CAN can(can_data);
    pcan = &can;

    measurer.init(&can);

    accelerometerOn();

    measurer.selfTest();

    while (1) {
        measurer.tick();

        Informator_can.inform(measurer.getClock());
        Informator_led.inform(measurer.getClock());

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
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 6;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_10TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN_Init 2 */

    /* USER CODE END CAN_Init 2 */
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
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ADDR_1_Pin ADDR_3_Pin */
    GPIO_InitStruct.Pin = ADDR_1_Pin | ADDR_3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ADDR_2_Pin ADDR_0_Pin */
    GPIO_InitStruct.Pin = ADDR_2_Pin | ADDR_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LED1_RGB_RED_Pin LED1_RGB_GREEN_Pin PB10 LED1_RGB_BLUE_Pin
                             B_ACCELEROMETER_Pin B_GYROSCOPE_Pin */
    GPIO_InitStruct.Pin = LED1_RGB_RED_Pin | LED1_RGB_GREEN_Pin | GPIO_PIN_10 | LED1_RGB_BLUE_Pin | B_ACCELEROMETER_Pin | B_GYROSCOPE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ADDR_4_Pin */
    GPIO_InitStruct.Pin = ADDR_4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADDR_4_GPIO_Port, &GPIO_InitStruct);

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
