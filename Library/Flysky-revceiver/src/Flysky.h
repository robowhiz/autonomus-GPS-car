#ifndef FLYSKY_H
#define FLYSKY_H

#include <Arduino.h>

#define MAX_NUM_CHANNELS 20

class FLYSKY
{
public:
    FLYSKY(uint8_t ppm_pin, uint8_t no_of_channels);
    void attachInterrupt();
    static void ppmInterrupt();
    uint16_t getPpmValue(uint8_t channel){
        return channel - 1 < _no_of_channels ? _ppmValues[channel - 1] : 0;
    };

private:
    uint8_t _ppm_pin;
    uint8_t _no_of_channels;
    volatile uint16_t _ppmValues[MAX_NUM_CHANNELS];
    volatile uint8_t _currentChannel;
    volatile uint32_t _lastTime;

    static FLYSKY* instance;
};

#endif