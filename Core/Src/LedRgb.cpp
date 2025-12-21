#include "LedRgb.h"

LED_RGB::LED_RGB(LED_RGB_Data &led) : led(led) {
    setColor(Color::OFF);
}

void LED_RGB::setColor(Color color) {
    state = color;
    setPin(led.r.GPIO_Port, led.r.GPIO_Pin, (color >> 2) & 1);
    setPin(led.g.GPIO_Port, led.g.GPIO_Pin, (color >> 1) & 1);
    setPin(led.b.GPIO_Port, led.b.GPIO_Pin, color & 1);
}

void LED_RGB::Off() {
    setColor(Color::OFF);
}

void LED_RGB::OK() {
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

void LED_RGB::OK(Color color) {
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
