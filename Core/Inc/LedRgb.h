#pragma once
#include "LedBase.h"

typedef struct LED_RGB_Data {
        LED_Color_Data r;
        LED_Color_Data g;
        LED_Color_Data b;
} LED_RGB_Data;

class LED_RGB : public LED_Base {
    private:
        LED_RGB_Data led;

    public:
        LED_RGB(LED_RGB_Data &led);

        void setColor(Color color);
        void Off();
        void OK();
        void OK(Color color);
};
