#pragma once
#include "LedBase.h"

class LED : public LED_Base {
    private:
        LED_Color_Data led;

    public:
        LED(LED_Color_Data &led);

        void setColor(Color color);
        void On();
        void Off();
};
