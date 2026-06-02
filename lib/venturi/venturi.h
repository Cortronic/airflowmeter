#ifndef VENTURI_H
#define VENTURI_H
#include <Arduino.h>

class Venturi {
  public:
    Venturi() {}
    Venturi(float inletDiameter, float throatDiameter, float dischargeCoefficient = 0.975);
    void  begin(float inletDiameter, float throatDiameter, float dischargeCoefficient); 
    float update(float deltaP,  float absPressurePa=0, float tempC=20.0, float humidityPct=50.0);
    float setRho(float absPressurePa, float tempC=20.0, float humidityPct=50.0);
    float getFlow(float deltaP);
    float getLastFlow () { return _flow; }
    float getSmoothedFlow() const { return _smoothedFlow; }
    void  setInletDiameter(float diameter);
    float getInletDiameter() const { return _inletDiameter; }
    void  setThroatDiameter(float diameter);
    float getThroatDiameter() const { return _throatDiameter; }
    void  setDischargeCoefficient(float cd) { _dischargeCoefficient = cd; }
    float getDischargeCoefficient() const {return _dischargeCoefficient; }
    void  setSmoothedFlowFactor(float factor) { _smoothedFlowFactor = factor; }
    float getSmoothedFlowFactor() const { return _smoothedFlowFactor; }

  private:
    float _inletDiameter;        // D
    float _throatDiameter;       // d
    float _dischargeCoefficient; // Cd
    float _betaCoefficient;      // Cb = 1 - (d/D)^4
    float _areaThroat;           // A2 = π * (d/2)^2
    float _rho = 1.225;          // Air density in kg/m³ (at standard conditions)
    float _flow = 0.0;           // Current flow in m³/h
    float _smoothedFlow = 0.0;   // For flow smoothing (optional)
    float _smoothedFlowFactor = 0.1; // Smoothing factor for exponential smoothing
};

#endif // VENTURI_H