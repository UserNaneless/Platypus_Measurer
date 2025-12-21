#include "CAN.h"

CAN::CAN(CAN_Init data) {
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
        } else {
            data.led->setColor(Color::OFF);
        };
    }
}

bool CAN::GetMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *data) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rxHeader, data) == HAL_OK) {
        return true;
    };

    return false;
}

bool CAN::SendMessage(uint8_t *data, uint32_t dlc) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
        return false;

    CAN_TxHeaderTypeDef txHeader;
    txHeader.DLC = dlc;

    return HAL_CAN_AddTxMessage(hcan, &txHeader, data, &mailbox) == HAL_OK;
}

bool CAN::compareCAN(CAN_HandleTypeDef *hcan) {
    return hcan == this->hcan;
}
