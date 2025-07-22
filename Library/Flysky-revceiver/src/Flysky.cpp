#include "Flysky.h"

FLYSKY* FLYSKY::instance = nullptr;

FLYSKY::FLYSKY(uint8_t ppm_pin, uint8_t no_of_channels)
    : _ppm_pin(ppm_pin), _no_of_channels(no_of_channels), _currentChannel(0), _lastTime(0)
{
    instance = this;
}

void FLYSKY::attachInterrupt()
{   pinMode(_ppm_pin, INPUT);
    ::attachInterrupt(digitalPinToInterrupt(_ppm_pin), FLYSKY::ppmInterrupt, RISING);
}

void FLYSKY::ppmInterrupt()
{
    uint32_t currentTime = micros();
    uint32_t pulseLength = currentTime - instance->_lastTime;
    instance->_lastTime = currentTime;

    if (pulseLength >= 3000) {
        instance->_currentChannel = 0;
    } else if (instance->_currentChannel < instance->_no_of_channels) {
        instance->_ppmValues[instance->_currentChannel] = pulseLength;
        instance->_currentChannel++;
    }
}