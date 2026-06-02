#include "PidControllerV2.h"

PidControllerV2::PidControllerV2(float kp, float ki, float kd, float dt, float minOutput, float maxOutput)
  : _emaFilter(new AdaptiveEmaFilter(0.01f, 0.1f))
  , _outputFilter(new TemaFilter(0.1f))
  , _kp(kp)
  , _ki(ki)
  , _kd(kd)
  , _dt(dt)
  , _filteredInput(0)
  , _setpoint(0)
  , _lastOutput(0)
  , _integral(0)
  , _minOutput(minOutput)
  , _maxOutput(maxOutput)
  , _controllerDirection(DIRECT) {
}

PidControllerV2::~PidControllerV2() {
  delete _emaFilter;
  delete _outputFilter;
}
 
// In this update function we use EMA low-pass filtering on the input to reduce noise, 
// and we also apply error scaling and squaring to make the controller
// less aggressive for small errors while still allowing for strong responses to larger errors. 
// The derivative term is calculated based on the change in the filtered input, 
// and we use a simple moving average to smooth the derivative term over several updates to further reduce noise. 
// We also include an output trend limit to prevent large jumps in the output, and optional output smoothing for a more gradual response.
float PidControllerV2::update(float input) {
  
  if (_controllerDirection == REVERSE) {
    input = -input; // Invert input voor reverse mode
  }
  
  // 1. Update the adaptive EMA filter with the new input value and get the filtered value
  float filteredInput = _emaFilter->update(input); 

  // 2. Calculate the error based on the input for the I action
  float error = _setpoint - input;

  // 3. Calculate the error based on the filtered input for the P action (and D action later)
  float filteredError = _setpoint - filteredInput;
  
  // 4. Calculate P action
  float pTerm = _kp * filteredError;
  
  // 5. Calculate D action based on the change in the filtered input, not the raw input, to reduce noise.
  float dTerm = - ((filteredInput - _filteredInput) / _dt) * _kd; // Derivative based on filtered input
  _filteredInput = filteredInput; // Update _filteredInput for the next derivative calculation
  
  // 6. Integral calculation with Clamping Anti-Windup
  if (_ki > 0) {
    float potentialIntegral = _integral + (filteredError * _ki * _dt);
    float outputPreClamp = pTerm + potentialIntegral + dTerm;

    // Check for saturation (PWM outside limits) and whether the error is in the same direction as the saturation
    bool saturated = (outputPreClamp > _maxOutput  || outputPreClamp < _minOutput);
    bool sameDirection = (filteredError > 0 && outputPreClamp > _maxOutput) || (filteredError < 0 && outputPreClamp < _minOutput); 

    // Only integrate if we are NOT saturated in the direction of the error
    if (!(saturated && sameDirection)) {
      _integral = potentialIntegral;
    }
  } else if (_integral > 0.01 || _integral < -0.01) {
    _integral *= 0.9; // Exponentiële decay of the integral if it is not being used
  }

  // 7. Output calculation and clamping
  float output = pTerm + _integral + dTerm;
  output = output > _maxOutput ? _maxOutput : (output < _minOutput ? _minOutput : output);

  // 8. Apply TEMA smoothing to the output
  output = _outputFilter->update(output); 

  _lastOutput = output; // Store the last output for the next update  
  return output;  
}

void PidControllerV2::setTunings(float kp, float ki, float kd) { 
  
  if (kp < 0 || ki < 0 || kd < 0) 
    return; // Don't allow negative values
  _kp = kp; 
  _ki = ki;
  _kd = kd; 
}

void PidControllerV2::reset() {
  _integral = 0; // Reset integral term
  _emaFilter->reset(); // Reset the adaptive EMA filter
  _filteredInput = 0; // Reset filtered input
}

void PidControllerV2::setControllerDirection(Direction dir) { 
  if (dir != _controllerDirection) {
    _controllerDirection = dir;
    reset(); // Reset integral and filter to prevent abrupt changes
  } 
}
