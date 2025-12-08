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
#include "Led.h"
#include "LedBase.h"
#include <cstring>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define wait HAL_Delay
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

#define ACCELEROMETER_ON \
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_RESET);
#define ACCELEROMETER_OFF \
    HAL_GPIO_WritePin(B_ACCELEROMETER_GPIO_Port, B_ACCELEROMETER_Pin, GPIO_PIN_SET);

#define GYROSCOPE_ON \
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_RESET);
#define GYROSCOPE_OFF \
    HAL_GPIO_WritePin(B_GYROSCOPE_GPIO_Port, B_GYROSCOPE_Pin, GPIO_PIN_SET);
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

void writeToReg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg, value };
    tx[0] &= 0x7F;
    uint8_t rx[2];
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 1000);
    STOP;
}

uint8_t readFromReg(uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0 };
    uint8_t rx[2];
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 1000);
    STOP;
    return rx[1];
}

void readFromReg(uint8_t reg, uint8_t *data, size_t size) {
    uint8_t tx[1 + size];
    tx[0] = (uint8_t)(reg | 0x80);
    for (size_t i = 1; i < size; i++) {
        tx[i] = 0;
    }
    uint8_t rx[1 + size];
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, size + 1, 1000);
    STOP;
    memcpy(data, rx + 1, size);
}

void writeToReg_Acc(uint8_t reg, uint8_t value) {
    ACCELEROMETER;
    writeToReg(reg, value);
}

uint8_t readFromReg_Acc(uint8_t reg) {
    ACCELEROMETER;
    uint8_t val = readFromReg(reg);
    return val;
}

void readFromReg_Acc(uint8_t reg, uint8_t *data, size_t size) {
    ACCELEROMETER;
    readFromReg(reg, data, size);
}

void accelerometerOn() {
    writeToReg_Acc(ACC_SOFT_RESET, 0xB6);

    wait(100);

    writeToReg_Acc(ACC_PWR_CONF, 0x00);
    writeToReg_Acc(ACC_POWER_CTL, 0x04);

    wait(100);
}

void accelerometerBIT() {
    //+-24g
    writeToReg_Acc(ACC_RANGE, 0x03);

    // 1.6khz
    writeToReg_Acc(ACC_CONF, 0xA7);

    wait(2);

    // self test polarity +
    writeToReg_Acc(ACC_SELF_TEST, 0x0D);

    wait(50);
    uint8_t positive[6] = { 0 };
    uint8_t pRange = 0;
    uint8_t negative[6] = { 0 };
    uint8_t nRange = 0;

    readFromReg_Acc(ACC_X_LSB, positive, 6);
    // HAL_SPI_TransmitReceive(&hspi1, &tx, positive, 6, 1000);

    pRange = readFromReg_Acc(ACC_RANGE);

    // self test polarity -
    writeToReg_Acc(ACC_SELF_TEST, 0x09);

    wait(50);

    readFromReg_Acc(ACC_X_LSB, negative, 6);
    // HAL_SPI_TransmitReceive(&hspi1, &tx, negative, 6, 1000);

    nRange = readFromReg_Acc(ACC_RANGE);

    // disable self test
    writeToReg_Acc(ACC_SELF_TEST, 0x00);

    int pAccel[3] = {
        (int)((positive[1] << 8) | positive[0]),
        (int)((positive[3] << 8) | positive[2]),
        (int)((positive[5] << 8) | positive[4]),
    };

    int nAccel[3] = {
        (int)((negative[1] << 8) | negative[0]),
        (int)((negative[3] << 8) | negative[2]),
        (int)((negative[5] << 8) | negative[4]),
    };

    wait(50);
}

void writeToReg_Gyro(uint8_t reg, uint8_t value) {
    GYROSCOPE;
    writeToReg(reg, value);
}

uint8_t readFromReg_Gyro(uint8_t reg) {
    GYROSCOPE;
    uint8_t val = readFromReg(reg);
    return val;
}

void readFromReg_Gyro(uint8_t reg, uint8_t *data, size_t size) {
    GYROSCOPE;
    readFromReg(reg, data, size);
}

void gyroscopeRead() {
    uint8_t data[6] = {};
    uint8_t chipID = readFromReg_Gyro(GYRO_CHIP_ID);

    readFromReg_Gyro(GYRO_RATE_X_LSB, data, 6);

    float x = (int16_t)((data[1] << 8) | data[0]) / 16.384f;
    float y = (int16_t)((data[3] << 8) | data[2]) / 16.384f;
    float z = (int16_t)((data[5] << 8) | data[4]) / 16.384f;
}

void accelerometerRead() {
    uint8_t data[6] = {};
    uint8_t chipID_discard = readFromReg_Acc(ACC_CHIP_ID);
    uint8_t chipID = readFromReg_Acc(ACC_CHIP_ID);
    uint8_t pConf = readFromReg_Acc(ACC_PWR_CONF);

    readFromReg_Acc(ACC_X_LSB, data, 6);

    uint8_t error = readFromReg_Acc(0x02);

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
}

typedef struct {
        CAN_HandleTypeDef *hcan;
        uint32_t id;
        LED *led;
} CAN_Init;

class CAN {
    private:
        CAN_HandleTypeDef *hcan;
        uint32_t mailbox;

        CAN_TxHeaderTypeDef txHeader;

    public:
        CAN(CAN_Init data) {
            this->hcan = data.hcan;
            txHeader.StdId = data.id;
            txHeader.IDE = CAN_ID_STD;
            txHeader.RTR = CAN_RTR_DATA;

            if (HAL_CAN_Start(this->hcan) == HAL_OK) {
                data.led->setColor(Color::BLUE);
                wait(500);
                if (HAL_CAN_ActivateNotification(this->hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
                    data.led->setColor(Color::RED);
                    wait(1000);
                };
                data.led->setColor(Color::OFF);
            }
        }

        bool GetMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *data) {
            if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, data) == HAL_OK) {
                return true;
            };

            return false;
        }

        bool SendMessage(uint8_t *data, uint32_t dlc) {
            if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
                return false;

            CAN_TxHeaderTypeDef txHeader;
            txHeader.DLC = dlc;

            return HAL_CAN_AddTxMessage(hcan, &txHeader, data, &mailbox) == HAL_OK;
        }

        bool compareCAN(CAN_HandleTypeDef *hcan) {
            return hcan == this->hcan;
        }
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

bool isPinSet(GPIO_TypeDef *port, uint8_t pin) {
    return HAL_GPIO_ReadPin(port, pin);
}

class Measurer {
    private:
        CAN *can;

    public:
        Measurer() {
        }

        void init(CAN *can) {
            this->can = can;
        }

        uint32_t readAddr() {
            const bool addr[5] = {
                isPinSet(GPIOB, GPIO_PIN_0),
                isPinSet(GPIOB, GPIO_PIN_1),
                isPinSet(GPIOB, GPIO_PIN_2),
                isPinSet(GPIOB, GPIO_PIN_3),
                isPinSet(GPIOB, GPIO_PIN_4),
            };
        }
};

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
    writeToReg_Gyro(GYRO_SOFTRESET, 0xB6);
    wait(100);
    writeToReg_Gyro(GYRO_RANGE, 0x01);
    writeToReg_Gyro(GYRO_BANDWIDTH, 0x02);
    writeToReg_Gyro(GYRO_LPM1, 0x00);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    LED_Color_Data led_data = {
        .GPIO_Port = LED_GPIO_Port,
        .GPIO_Pin = LED_Pin
    };

    LED led(led_data);
    can_led = &led;

    CAN_Init can_data = {
        .hcan = &hcan,
        .id = 0x123,
        .led = &led
    };

    CAN can(can_data);

    accelerometerOn();
    while (1) {
        // accelerometerTest();
        // gyroscopeBIT();
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
    hcan.Init.Prescaler = 16;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
