#ifndef TEMA_FILTER_H
#define TEMA_FILTER_H
#include <math.h>

class TemaFilter {

  public:
    TemaFilter(float alpha)
    : _is_initialized(0) {
      setAlpha(alpha); // Initialize _alpha1, _alpha2 and _alpha3 based on the provided alpha
    }

    float update(float input) {
      if (!_is_initialized) {
        _ema1 = input;
        _ema2 = input;
        _ema3 = input;
        _is_initialized = 1;
        return input; // Return the raw input on the first call
      } else {
        _ema1 = _alpha1 * input + (1 - _alpha1) * _ema1;
        _ema2 = _alpha2 * _ema1 + (1 - _alpha2) * _ema2;
        _ema3 = _alpha3 * _ema2 + (1 - _alpha3) * _ema3;
        return (3 * _ema1) - (3 * _ema2) + _ema3; // Tema output
      } 
    }

    void reset() {
      _ema1 = 0.0;
      _ema2 = 0.0;
      _ema3 = 0.0;
      _is_initialized = 0;
    }

    void setAlpha(float alpha) {
      if (alpha < 0 || alpha > 1) {
        return; // Invalid alpha value, do not update
      }
      _alpha1 = alpha;
      // Calculate _alpha2 and _alpha3 to maintain the properties of the TEMA filter
      float r = powf(1.0f / alpha, 1.0f / 3.0f); // Calculate the common ratio for the geometric progression
      _alpha2 = r * _alpha1; 
      _alpha3 = r * _alpha2;
    }

    float getAlpha() const {
      return _alpha1;
    }

  private:
    float _alpha1;
    float _alpha2;
    float _alpha3;
    float _ema1 = 0.0;
    float _ema2 = 0.0;
    float _ema3 = 0.0;
    int _is_initialized = 0;
};

#endif // TEMA_FILTER_H