#pragma once

#define wait HAL_Delay

#include "stm32f1xx_hal.h"

typedef struct LED_Color_Data {
        GPIO_TypeDef *GPIO_Port;
        uint16_t GPIO_Pin;
} LED_Color_Data;

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

class LED_Base {
    protected:
        Color state = Color::OFF;

        void setPin(GPIO_TypeDef *port, uint16_t pin, bool state);

    public:
        virtual void setColor(Color color);

        virtual ~LED_Base() = default;

        virtual void Off();

        virtual void OK();
};
