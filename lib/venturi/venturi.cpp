#include "venturi.h"


Venturi::Venturi(float inletDiameter, float throatDiameter, float dischargeCoefficient) {
  begin(inletDiameter, throatDiameter, dischargeCoefficient);
}

void Venturi::begin(float inletDiameter, float throatDiameter, float dischargeCoefficient) {
  _inletDiameter = inletDiameter;
  _throatDiameter = throatDiameter;
  _dischargeCoefficient = dischargeCoefficient;
  _betaCoefficient = 1 - pow(throatDiameter / inletDiameter, 4);
  _areaThroat = (PI * pow(throatDiameter, 2)) / 4.0;
}

float Venturi::update(float deltaP,  float absPressurePa, float tempC, float humidityPct) {
  if (absPressurePa > 0) {   
    setRho(absPressurePa, tempC, humidityPct);
  }
  if (deltaP <= 0) {
    _smoothedFlow = _flow = 0.0;
    return 0.0;
  }
  if (_smoothedFlow == 0.0) {
    _smoothedFlow = _flow = getFlow(deltaP); // Initialize smoothed flow with the first reading
  } else {
    _flow = getFlow(deltaP);
    // Optional: Implement flow smoothing here if needed
    _smoothedFlow = _flow * _smoothedFlowFactor + _smoothedFlow * (1.0 - _smoothedFlowFactor); // Simple exponential smoothing
  }  
  return _flow;
}

float Venturi::setRho(float absPressurePa, float tempC, float humidityPct) {
  // Calculate the density of humid air using the ideal gas law and accounting for humidity
  float R = 287.05; // Specific gas constant for dry air in J/(kg·K)
  float T = tempC + 273.15; // Convert temperature to Kelvin
  float P = absPressurePa; // Absolute pressure in Pascals

  // Calculate the saturation vapor pressure using the Magnus formula
  float Es = 610.94 * exp((17.625 * tempC) / (tempC + 243.04)); // Saturation vapor pressure in Pa
  float E = Es * (humidityPct / 100.0); // Actual vapor pressure in Pa

  // Calculate the density of humid air
  _rho = (P - E) / (R * T) + E / (461.495 * T); // 461.495 is the specific gas constant for water vapor
  return _rho;  // return the density in kg/m³
}

float Venturi::getFlow(float deltaP) {
  if (deltaP <= 0) return 0.0;

  // Calculate the velocity at the throat using Bernoulli's equation
  float velocityThroat = sqrt((2 * deltaP) / (_rho * _betaCoefficient));
      
  // Calculate the volumetric flow rate using the discharge coefficient and area of the throat
  float flowM3s = _dischargeCoefficient * _areaThroat * velocityThroat;

  return flowM3s * 3600.0; // Convert from m³/s to m³/h
}

void Venturi::setInletDiameter(float diameter) {
  _inletDiameter = diameter;
  _betaCoefficient = 1 - pow(_throatDiameter / _inletDiameter, 4);
}

void Venturi::setThroatDiameter(float diameter) {
  _throatDiameter = diameter;
  _betaCoefficient = 1 - pow(_throatDiameter / _inletDiameter, 4);
  _areaThroat = (PI * pow(_throatDiameter, 2)) / 4.0;
}