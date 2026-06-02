#ifndef DEMAFILTER_H
#define DEMAFILTER_H

class DemaFilter {

  public:
    DemaFilter(float alpha)
      : _alpha(alpha) {}
    
    float update(float input) {
      if (!_is_initialized) {
        _ema1 = input;
        _ema2 = input;
        _is_initialized = 1;
        return input; // Return the raw input on the first call
      } else {
        _ema1 = _alpha * input + (1 - _alpha) * _ema1;
        _ema2 = _alpha * _ema1 + (1 - _alpha) * _ema2;
        return (2 * _ema1) - _ema2; // DEMA output
      } 
    }

    void reset() {
      _ema1 = 0.0;
      _ema2 = 0.0;
      _is_initialized = 0;
    }

    void setAlpha(float alpha) {
      if (alpha < 0 || alpha > 1) {
        return; // Invalid alpha value, do not update
      }
      _alpha = alpha;
    }

    float getAlpha() const {
      return _alpha;
    }

  private:
    float _alpha;
    float _ema1 = 0.0;
    float _ema2 = 0.0;
    int _is_initialized = 0;
};

#endif // DEMAFILTER_H