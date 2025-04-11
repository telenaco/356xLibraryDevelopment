# MCP356x Calibration Examples

This folder contains examples for calibrating MCP356x ADCs with various load cell configurations, from simple single-axis to complex 6-axis force/torque sensors.

## Purpose

The calibration examples help you:
- Understand different calibration methods for load cells
- Implement accurate measurement systems
- Record and store calibration data
- Apply calibration matrices for multi-axis sensors

## Basic Calibration

### mcp356x_calibration_basic
Basic ADC calibration for single load cells:
- Captures raw and filtered readings for calibration analysis
- Simple serial output of readings
- Companion Python script for generating calibration coefficients
- High-speed sampling for accurate calibration data collection

### mcp356x_calibration_force_basic
Simple example showing scale factor calibration:
- Single-axis load cell configuration
- Basic tare operation
- Simple scale factor application
- Real-time display of readings

### mcp356x_calibration_all_channels
Multi-channel calibration system:
- Calibrates up to four channels simultaneously (DIFF_A through DIFF_D)
- Interactive calibration via serial interface
- Storage of calibration points
- CSV output of complete calibration dataset

### mcp356x_calibration_sbeam
Specialized calibration for S-beam load cells:
- Optimized for industrial S-beam load cell sensors
- Multiple processing paths (raw, filtered, calibrated)
- Real-time performance metrics
- Support for tare operations via serial commands

## 3-Axis Load Cell Calibration

### mcp356x_triaxial_load_cell_calibration
Complete example for 3-axis load cell calibration:
- Matrix-based calibration for cross-axis compensation
- Visualization of calibrated force vectors
- Interactive calibration procedure
- Detailed diagnostic outputs

## 6-Axis Force/Torque Sensor Calibration

### mcp356x_calibration_6axis_forces
Advanced calibration for 6-axis force/torque sensors:
- Integrates four 3-axis load cells into a 6-axis sensor
- Full matrix calibration for all six degrees of freedom
- High-precision force and torque measurements
- Compensation for mechanical coupling between axes

### mcp356x_calibration_6axis_recorder
Interactive utility for recording calibration data:
- Serial interface for controlling calibration process
- Storage of calibration points for later analysis
- Supports multiple calibration matrices
- Visualization of force and torque readings

## Telemetry

### mcp356x_telemetry_12_cells
Streaming data format for telemetry:
- Reads data from 12 load cells
- Compatible with Telemetry Viewer application
- Configurable sample intervals
- Support for tare commands via serial interface

## File Structure

```
Calibration/
├── mcp356x_calibration_basic/
│   ├── mcp356x_calibration_basic.cpp
│   ├── mcp356x_calibration_basic.csv
│   └── mcp356x_calibration_basic.py
├── mcp356x_telemetry_12_cells/
│   ├── mcp356x_telemetry_12_cells.cpp
│   ├── telemetryViewerConfiguration.txt
│   └── telemetryViewerConfiguration - connection 0 - UART COM3.csv
├── mcp356x_calibration_6axis_forces/
│   ├── mcp356x_calibration_6axis_forces.cpp
│   ├── mcp356x_calibration_6axis_recorder.cpp
│   ├── TelemetryConfiguration.txt
│   └── TelemetryConfiguration - connection 0 - UART COM3.csv
├── mcp356x_calibration_all_channels.cpp
├── mcp356x_calibration_force_basic.cpp
├── mcp356x_calibration_sbeam.cpp
└── mcp356x_triaxial_load_cell_calibration.cpp
```

## Calibration Methods

The examples demonstrate several calibration approaches:

1. **Scale Factor Calibration**  
   Simple multiplication factor for converting raw readings to force units.
   ```cpp
   calibratedValue = rawValue * scaleFactor;
   ```

2. **Linear Calibration**  
   Using a linear equation for more accurate results:
   ```cpp
   calibratedValue = rawValue * slope + intercept;
   ```

3. **Polynomial Calibration**  
   Using a second-order polynomial for improved accuracy:
   ```cpp
   calibratedValue = a*rawValue² + b*rawValue + c;
   ```

4. **Matrix Calibration**  
   For multi-axis sensors with cross-axis compensation:
   ```cpp
   [Fx]   [c11 c12 c13] [raw_x]
   [Fy] = [c21 c22 c23] [raw_y]
   [Fz]   [c31 c32 c33] [raw_z]
   ```

5. **6-Axis Matrix Calibration**  
   Full calibration for force/torque sensors:
   ```cpp
   [Fx]   [c11 c12 ... c16] [raw_1]
   [Fy]   [c21 c22 ... c26] [raw_2]
   [Fz]   [c31 c32 ... c36] [raw_3]
   [Mx] = [c41 c42 ... c46] [raw_4]
   [My]   [c51 c52 ... c56] [raw_5]
   [Mz]   [c61 c62 ... c66] [raw_6]
   ```

## Hardware Setup

Default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13

For multi-ADC examples:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC0 | 7 | 6 |
| ADC1 | 4 | 5 |
| ADC2 | 2 | 3 |

## Getting Started

1. Begin with the basic calibration examples to understand the fundamentals
2. For single-axis load cells, use `mcp356x_calibration_force_basic`
3. For multi-axis sensors, use the appropriate specialized examples
4. Connect your hardware according to the pin definitions
5. Upload the example and follow calibration instructions via serial monitor at 115200 baud