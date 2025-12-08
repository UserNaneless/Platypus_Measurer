#include "LedBase.h"

void LED_Base::setPin(GPIO_TypeDef *port, uint16_t pin, bool state) {
    if (state)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void LED_Base::setColor(Color color) {
    state = color;
};

void LED_Base::Off() {
    setColor(Color::OFF);
};

void LED_Base::OK() {
    setColor(state);
    wait(300);
    Off();
    wait(300);
    setColor(state);
    wait(300);
    Off();
    wait(300);
    setColor(state);
    wait(300);
    Off();
}
