#include "fan.h"

Fan::Fan(int pin, int chan, int freq, int pwmResolution, int ditherResolution) 
  : _enabled(false)
  , _pin(pin)
  , _chan(chan)
  , _freq(freq)
  , _pwmResolution(pwmResolution)
  , _ditherResolution(ditherResolution)
  , _speed(0)
  , _maxSpeed(((1 << pwmResolution) - 1) << ditherResolution)
  , _ditherCounter(0)
  , _ditherSpeed(0) {}

void Fan::begin() {
  // Configure PWM for the fan
  ledcSetup(_chan, _freq, _pwmResolution);
  ledcAttachPin(_pin, _chan);
  _enabled = true;
  setSpeed(0); // Start with the fan off
}

// This function should be called periodically (e.g., every 100ms) if dithering is enabled.
IRAM_ATTR void Fan::handleDither() {
  if (_enabled && _ditherResolution > 0) {
    // For duty cycle between _speed and _speed + 1 based on _ditherCounter and _ditherSpeed
    _ditherCounter = (_ditherCounter + 1) % (1 << _ditherResolution);
    uint16_t ditheredSpeed = _speed + (_ditherCounter < _ditherSpeed ? 1 : 0);
    ledcWrite(_chan, ditheredSpeed);
  }
}

void Fan::setSpeed(uint16_t speed) {
  if (_enabled) {
    if (_ditherResolution > 0) {
      // If dithering is enabled, we will handle the actual speed in the setDither function
      speed = speed >= _maxSpeed ? _maxSpeed : speed; 
      _speed = speed >> _ditherResolution;
      _ditherSpeed = speed & ((1 << _ditherResolution) - 1);
    } else {
      // If dithering is not enabled, set the speed directly
      _speed = speed >= _maxSpeed ? _maxSpeed : speed; // Cap speed to max value
      ledcWrite(_chan, _speed);
    }
  }
}

void Fan::setDither(uint8_t resolution) {
  // Dithering is not directly supported by the ESP32 LEDC hardware, but it is here implemented in software if needed.
  // This function is a placeholder for enabling/disabling dithering if you choose to use it.
  _maxSpeed = ((1 << _pwmResolution) - 1) << resolution; // Update max speed based on new dither resolution
  if (resolution > 0) {
    // Implement dithering logic here if desired
    _ditherResolution = resolution;
    _ditherCounter = 0;
  } else {
    // Disable dithering logic here if implemented
    _ditherResolution = 0;
    // Ensure the fan speed is set to the current speed without dithering
    if (_enabled) {
      ledcWrite(_chan, _speed);
    }
  }
}

int Fan::getSpeed() {
    return _speed;
}