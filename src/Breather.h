#ifndef BREATHER_H
#define BREATHER_H

#include <Arduino.h>

class Breather
{
private:
    int pin;
    int maxBrightness;
    unsigned long phaseShift;
    unsigned long halfPeriod;
    unsigned long previousTime;

public:
    enum BreatheState
    {
        OFF,
        BREATHING_CONTINUOUSLY,
        BREATHE_ONCE
    } state;

    Breather(int pin);

    void initialize();
    void update();
    void startBreathing(int cycleDuration, unsigned long delay = 0);
    void stopBreathing();
    void breatheOnce(int cycleDuration, unsigned long startDelay = 0);
    String getName() const;
};

#endif // BREATHER_H
