# 3D-printed Venturi Flow Meter

<p align="left">
  <img src="airflowmeter.png" width="200" height= "300">
</p>


# Zero-Pressure Compensated Airflow Meter & Calibration Rig
This project provides an open-source, high-precision solution for measuring air flow in ventilation systems (such as HRV/MVHR units). For approximately €100, you can build an instrument that rivals the accuracy of professional meters costing thousands of euros.

# 1. Project Overview
The system consists of two configurations running on the same ESP32 architecture:

The Flow Meter (DUT): Uses an active compensation fan to eliminate its own flow resistance (zero-pressure measurement).

The Test Rig (Reference): A modular 125mm wind tunnel featuring a calibrated Venturi section for reference measurements.