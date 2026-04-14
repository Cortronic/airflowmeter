#ifndef FAN_H
#define FAN_H
#include <Arduino.h>
#include <driver/ledc.h>

#define FAN_PWM_CHAN LEDC_CHANNEL_0
#define FAN_PWM_FREQ 25000    // 25 kHz frequency for computer fans
#define FAN_PWM_RESOLUTION 10 // 10-bit resolution (0-1023)
#define FAN_DITHER_RESOLUTION 2 // 2 bits of dithering for finer control

class Fan {
public:
    Fan(int pin, int chan = FAN_PWM_CHAN, int freq = FAN_PWM_FREQ, int pwmResolution = FAN_PWM_RESOLUTION, int ditherResolution = FAN_DITHER_RESOLUTION);
    void begin();
    void IRAM_ATTR handleDither(); // should be periodically called to if dithering is enabled
    void setSpeed(uint16_t speed);
    void setDither(uint8_t resolution);
    int getSpeed();

private:
    bool     _enabled;
    uint8_t  _pin;
    uint8_t  _chan;
    uint32_t _freq;
    uint8_t  _pwmResolution;
    uint8_t  _ditherResolution;
    uint16_t _speed;
    uint16_t _maxSpeed;
    uint16_t _ditherCounter;
    uint16_t _ditherSpeed;
};
#endif