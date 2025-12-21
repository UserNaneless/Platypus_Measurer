#include "LedRgb.h"
#include "stm32f1xx_hal_can.h"

typedef struct {
        CAN_HandleTypeDef *hcan;
        uint32_t id;
        LED_RGB *led;
} CAN_Init;

class CAN {
    private:
        CAN_HandleTypeDef *hcan;
        uint32_t mailbox;

        CAN_TxHeaderTypeDef txHeader;

    public:
        CAN(CAN_Init data);
        bool GetMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *data);
        bool SendMessage(uint8_t *data, uint32_t dlc);
        bool compareCAN(CAN_HandleTypeDef *hcan);
};
