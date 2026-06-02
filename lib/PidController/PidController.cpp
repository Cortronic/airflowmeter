#include "PidController.h"

PidController::PidController(double kp, double ki, double kd, double dt, double minOutput, double maxOutput)
  : _outputFilter(new TemaFilter(0.1f))
  , _kp(kp)
  , _ki(ki)
  , _kd(kd)
  , _dt(dt)
  , _alpha(0.1) // Example value for input low-pass filtering
  , _scale(0.5) // Example value for error scaling
  , _filteredInput(0)
  , _setpoint(0)
  , _lastInput(0)
  , _lastOutput(0)
  , _outputTrendLimit(1.0) // Example value for output trend limit
  , _integral(0)
  , _derivative(0)
  , _sumDerivative(0)
  , _minOutput(minOutput)
  , _maxOutput(maxOutput)
  , _derivativeSmoothingCount(10)
  , _derivativeSmoothingCounter(0)
  , _controllerDirection(DIRECT) {
}

PidController::~PidController() {
  delete _outputFilter;
}
 

float PidController::update(double input) {
  
  if (_controllerDirection == REVERSE) {
    input = -input; // Invert input voor reverse mode
  }
  double error = _setpoint - input; // Calculate the error based on the raw input for better responsiveness, but we will use the filtered input for the P and D calculations to reduce noise.

  // In this update function we use simple low-pass filtering on the input before we do the PID calculations.
  double lastFilteredInput = _filteredInput;
  
  // 1. Filter the input with a simple low-pass filter to reduce noise before doing the PID calculations
  _filteredInput = _alpha * input  +  (1 - _alpha) * _filteredInput; // Simple low-pass filter on the input
  double filteredError = _setpoint - _filteredInput;

  // 2. Scale the error first back to a usable range, otherwise squaring it can become too large.
  // Adjust this factor based on the typical error values in your application.
  double scaledError = filteredError * _scale;
  
  // 3. Square the error to make the controller less aggressive for small deviations, but keep the direction of the error
  double squaredError = scaledError > 0 ? scaledError * scaledError : - (scaledError * scaledError);
  
  // 4. Calculate P action
  double pTerm = _kp * squaredError;
  
  // 5. Calculate D action based on the change in the filtered input, not the raw input, to reduce noise.
  double dTerm = 0; // Initialiseer dTerm
  if (_kd > 0) {
    _derivativeSmoothingCounter = (_derivativeSmoothingCounter + 1) % _derivativeSmoothingCount; // Update the counter for derivative calculation
    _sumDerivative += input - _lastInput; // Accumulate the change in input for smoothing
    _lastInput = input; // Store the last input for the next derivative calculation
    
    if (_derivativeSmoothingCounter == 0) { // Only calculate derivative every _derivativeSmoothingCount updates to reduce noise
      //double dInput = _filteredInput - lastFilteredInput; // Change in filtered input
      double dInput = _sumDerivative / _derivativeSmoothingCount; // smoothed change in input over the last _derivativeSmoothingCount updates
      double dInputScaled = dInput * _scale; // Scale the change in input with the same factor as the error
      double dInputSquared = dInputScaled > 0 ? dInputScaled * dInputScaled : - (dInputScaled * dInputScaled); // Square the change in input, keep the direction
      dTerm = _derivative = _kd * dInputSquared / (_dt * _derivativeSmoothingCount); // Derivative of the filtered, scaled, and squared dInput
      _sumDerivative = 0; // Reset the sum of derivatives
    } else {
      dTerm = _derivative; // Use the last calculated derivative term for the intermediate updates
    }
  }

  // 6. Integral calculation with Clamping Anti-Windup
  if (_ki > 0) {
    double potentialIntegral = _integral + (error * _ki * _dt);
    double outputPreClamp = pTerm + potentialIntegral - dTerm;

    // Check for saturation (PWM outside limits) and whether the error is in the same direction as the saturation
    bool saturated = (outputPreClamp > _maxOutput  || outputPreClamp < _minOutput);
    bool sameDirection = (error > 0 && outputPreClamp > _maxOutput) || (error < 0 && outputPreClamp < _minOutput); 

    // Only integrate if we are NOT saturated in the direction of the error
    if (!(saturated && sameDirection)) {
      _integral = potentialIntegral;
    }
  } else if (_integral > 0.01 || _integral < -0.01) {
    _integral *= 0.9; // Exponentiële decay of the integral if it is not being used
  }

  // 7. Final output calculation and clamping
  double output = pTerm + _integral - dTerm;
  output = output > _maxOutput ? _maxOutput : (output < _minOutput ? _minOutput : output);

  // 8. Output trend clamping to prevent large jumps in the output
  output = output - _lastOutput > _outputTrendLimit ?
    _lastOutput + _outputTrendLimit :
    (output - _lastOutput < -_outputTrendLimit ? _lastOutput - _outputTrendLimit : output);
    
  // 9. Smooth the output with the TEMA filter to further reduce noise and prevent aggressive changes in the output
  _lastOutput = _outputFilter->update(output);
  return _lastOutput;  
}

void PidController::setAlpha(double alpha) {
  _alpha = alpha; // Set the alpha value for the input low-pass filter
}

double PidController::getAlpha() const { 
  return _alpha;
}

void PidController::setTunings(double kp, double ki, double kd) { 
  
  if (kp < 0 || ki < 0 || kd < 0) 
    return; // Don't allow negative values
  
  _kp = kp; 
  _ki = ki; 
  
  if (kd != _kd) {
    _kd = kd; // Update the derivative gain
    _derivative = 0; // Reset the derivative term to prevent abrupt changes
    _sumDerivative = 0; // Reset the sum of derivatives for smoothing
    _derivativeSmoothingCounter = 0; // Reset the counter for derivative calculation
  }
}

void PidController::setControllerDirection(Direction dir) { 
  if (dir != _controllerDirection) {
    _controllerDirection = dir;
    reset(); // Reset integral and filter to prevent abrupt changes
  } 
}
