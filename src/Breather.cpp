#include "Breather.h"

Breather::Breather(int pin)
    : pin(pin), maxBrightness(255), phaseShift(0), halfPeriod(0), previousTime(0), state(OFF)
{
}

void Breather::initialize()
{
    pinMode(pin, OUTPUT);
    analogWrite(pin, 0); // LED OFF for a clear start
    phaseShift = 0;      // Reset phase shift
}

void Breather::update()
{
    if (state != OFF)
    {
        unsigned long time = (millis() - phaseShift) % (2 * halfPeriod);

        if (state == BREATHE_ONCE && previousTime > time) // means it has looped over
        {
            state = OFF;
            analogWrite(pin, 0); // LED OFF
            Serial.println(getName() + " finished a cycle and set to OFF state.");
            return;
        }

        int brightness;
        if (time < halfPeriod)
        {
            brightness = maxBrightness * time / halfPeriod;
        }
        else
        {
            brightness = maxBrightness * (2 * halfPeriod - time) / halfPeriod;
        }
        analogWrite(pin, brightness); // Adjust the brightness

        previousTime = time;
    }
    else
    {
        analogWrite(pin, 0); // LED OFF
    }
}

void Breather::startBreathing(int cycleDuration, unsigned long delay)
{
    state = BREATHING_CONTINUOUSLY;
    halfPeriod = cycleDuration / 2;
    phaseShift = millis() - delay;
}

void Breather::stopBreathing()
{
    state = OFF;
    analogWrite(pin, 0); // LED OFF
}

void Breather::breatheOnce(int cycleDuration, unsigned long startDelay)
{
    analogWrite(pin, 0); // Ensure the LED starts from 0
    state = BREATHE_ONCE;
    halfPeriod = cycleDuration / 2;
    phaseShift = millis() + startDelay - halfPeriod; // Adjust phaseShift to account for delay
    Serial.println(getName() + " set to BREATHE_ONCE state with a start delay of " + String(startDelay));
}

String Breather::getName() const
{
    return "Breather at pin " + String(pin);
}
