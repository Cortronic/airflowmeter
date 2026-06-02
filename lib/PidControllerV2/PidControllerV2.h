#ifndef PIDCONTROLLER_V2_H
#define PIDCONTROLLER_V2_H
#include <Arduino.h>
#include <AdaptiveEmaFilter.h>
#include <TemaFilter.h>

class PidControllerV2{
  
  public:
    typedef enum {
      DIRECT = 0,
      REVERSE = 1,
    } Direction;

    PidControllerV2(float kp, float ki, float kd, float dt, float minOutput, float maxOutput);
    ~PidControllerV2();
    float update(float input); // should be called every delta t seconds.
    void reset();
    float getOutput() const { return _lastOutput; }
    void setSetpoint(float sp) { _setpoint = sp; }
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minOut, float maxOut) { _minOutput = minOut; _maxOutput = maxOut; }
    void setSampleTime(float dt) { _dt = dt; } // in seconds
    void setControllerDirection(Direction dir);
    void setAlpha(float alpha) { _emaFilter->setBaseFilter(alpha); } // setter for alpha, the base filter strength for the input low-pass filter;
    float getAlpha() const { return _emaFilter->getBaseFilter(); } // getter for alpha
    void setSensitivity(float sensitivity) { _emaFilter->setSensitivity(sensitivity); } // setter for sensitivity, the error-scaling factor
    float getSensitivity() const { return _emaFilter->getSensitivity(); } // getter for sensitivity
    void setOutputSmoothing(float factor) { _outputFilter->setAlpha(factor); } // setter for output smoothing
    float getOutputSmoothing() const { return _outputFilter->getAlpha(); } // getter for output smoothing factor
    float getFilteredInput() const { return _filteredInput; }

  private:
    AdaptiveEmaFilter *_emaFilter; // Pointer to an instance of the adaptive EMA filter
    TemaFilter *_outputFilter; // Pointer to an instance of the TEMA filter
    float _kp, _ki, _kd;
    float _dt;
    float _filteredInput;
    float _setpoint;
    float _lastOutput;
    float _integral;
    float _minOutput;
    float _maxOutput;
    Direction _controllerDirection;   
};
#endif // PIDCONTROLLER_V2_H