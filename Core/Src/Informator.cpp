#include "Informator.h"

Informator::Informator(LED_RGB *led) {
    this->led = led;
}

Informator::Informator(LED *led) {
    this->led = led;
}

void Informator::enqueue(Informator_Signal signal) {
    signalQueue[current] = signal;
    current = (current + 1) % MAX_SIGNALS;
}

Informator_Signal &Informator::getSignal() {
    return signalQueue[currentToDo];
}

void Informator::next() {
    signalQueue[currentToDo].used = true;
    currentToDo = (currentToDo + 1) % MAX_SIGNALS;
}

void Informator::inform(uint32_t clock) {
    Informator_Signal &signal = getSignal();
    if (currentToDo == current && signal.duration == 0) {
        return;
    }

    if (!signal.active) {
        startTime = clock;
        signal.active = true;
        signal.state = true;
    }

    if (signal.active) {
        if (signal.blinkable) {

            if(signal.state) {
                led->setColor(signal.color);
                if(clock - startTime > signal.duration) {
                    signal.state = false;
                    startTime = clock;
                    signal.blink_count--;
                }
            } else {
                led->setColor(Color::OFF);
                if(clock - startTime > signal.blink_off_duration) {
                    signal.state = true;
                    startTime = clock;

                    if(signal.blink_count == 0) {
                        signal.active = false;
                        next();
                    }
                }
            }

        } else {
            if (clock - startTime > signal.duration) {
                signal.state = false;
                led->setColor(Color::OFF);
                if (clock - startTime > signal.duration + signal.blink_off_duration)
                    next();
            } else {
                led->setColor(signal.color);
                signal.state = true;
            }
        }
    }
}

void pushError(Informator *&informator) {
    Informator_Signal errorSignal = {
        .color = Color::RED,
        .duration = 100,

        .blinkable = true,
        .blink_count = 3,
        .blink_off_duration = 100
    };

    informator->enqueue(errorSignal);
}

void pushSuccess(Informator *&informator) {
    Informator_Signal successSignal = {
        .color = Color::GREEN,
        .duration = 100,

        .blinkable = true,
        .blink_count = 3,
        .blink_off_duration = 100
    };

    informator->enqueue(successSignal);
}

void pushInfo(Informator *&informator) {
    Informator_Signal infoSignal = {
        .color = Color::WHITE,
        .duration = 50,
        .blink_off_duration = 50
    };

    informator->enqueue(infoSignal);
}
