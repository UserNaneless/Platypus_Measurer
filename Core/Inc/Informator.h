#include "LedBase.h"
#include "LedRgb.h"
#include "Led.h"

#define MAX_SIGNALS 128

typedef struct {
        Color color = Color::OFF;
        uint32_t duration = 0;

        bool state = false;
        bool blinkable = false;

        bool active = false;
        bool used = false;

        uint8_t blink_count = 0;
        uint32_t blink_off_duration = 0;
} Informator_Signal;

class Informator {

    private:
        uint8_t current = 0;
        uint8_t currentToDo = 0;
        Informator_Signal signalQueue[MAX_SIGNALS] = {};

        Informator_Signal& getSignal();
        void next();

        LED_Base *led = nullptr;


        uint32_t startTime = 0;

    public:
        Informator(LED_RGB *led);
        Informator(LED *led);

        void inform(uint32_t clock);
        void enqueue(Informator_Signal signal);
};

void pushError(Informator *&informator);
void pushSuccess(Informator *&informator);
void pushInfo(Informator *&informator);
