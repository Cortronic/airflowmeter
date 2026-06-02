#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <TemaFilter.h>

class PidController{
  
  public:
    typedef enum {
      DIRECT = 0,
      REVERSE = 1,
    } Direction;

    PidController(double kp, double ki, double kd, double dt, double minOutput, double maxOutput);
    ~PidController();
    float update(double input); // should be called every delta t seconds.
    void reset() { _integral = 0; }
    double getOutput() const { return _lastOutput; }
    void setSetpoint(double sp) { _setpoint = sp; }
    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double minOut, double maxOut, double trendLimit) { _minOutput = minOut; _maxOutput = maxOut; _outputTrendLimit = trendLimit; }
    void setSampleTime(double dt) { _dt = dt; } // in seconds
    void setControllerDirection(Direction dir);
    void setAlpha(double alpha);
    double getAlpha() const;
    void setScale(double scale) { _scale = scale; } // setter for scale, the error-scaling factor
    double getScale() const { return _scale; } // getter for scale
    void setOutputTrendLimit(double trendLimit) { _outputTrendLimit = trendLimit; } // setter for output trend limit
    double getOutputTrendLimit() const { return _outputTrendLimit; } // setter for output trend limit
    void setDerivativeSmoothing(int count) { _derivativeSmoothingCount = count; } // setter for the number of updates to smooth the derivative term over
    int getDerivativeSmoothing() const { return _derivativeSmoothingCount; } // getter for the number of updates to smooth the derivative term over
    void setOutputSmoothing(double factor) { _outputFilter->setAlpha(factor); } // setter for output smoothing
    double getFilteredInput() const { return _filteredInput; }

  private:
    TemaFilter *_outputFilter; // Filter for smoothing the output
    double _kp, _ki, _kd;
    double _dt;
    double _alpha; // for input low-pass filtering.
    double _scale; // for error scaling, range 0-1, higher values makes the controller more aggressive.
    double _filteredInput;
    double _setpoint;
    double _lastInput; // For derivative calculation
    double _lastOutput;
    double _outputTrendLimit; // Max change in output per update to prevent aggressive changes.
    double _integral;
    double _derivative;
    double _sumDerivative; // For smoothing the derivative term
    double _minOutput;
    double _maxOutput;
    int _derivativeSmoothingCount; // Number of updates to smooth the derivative term over
    int _derivativeSmoothingCounter; // Counter for derivative calculation
    Direction _controllerDirection;   
};
#endif // PIDCONTROLLER_H