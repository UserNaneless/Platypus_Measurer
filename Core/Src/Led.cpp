#include "Led.h"

LED::LED(LED_Color_Data &led) : led(led) {
    setColor(Color::OFF);
}

void LED::setColor(Color color) {
    state = color;
    setPin(led.GPIO_Port, led.GPIO_Pin, color);
}

void LED::On() {
    setColor(Color::WHITE);
}

void LED::Off() {
    setColor(Color::OFF);
}
