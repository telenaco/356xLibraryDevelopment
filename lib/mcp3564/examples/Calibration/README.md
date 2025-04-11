# MCP356x Calibration Examples

This folder contains examples for calibrating MCP356x ADCs for various load cell configurations.

## Basic Calibration

### mcp356x_calibration_basic
Basic ADC calibration for single load cells. Captures raw and filtered readings for calibration analysis.

### mcp356x_calibration_force_basic
Simple example showing basic scale factor calibration and tare operation for a single load cell.

### mcp356x_calibration_all_channels
Calibrates multiple channels (DIFF_A through DIFF_D) of an MCP356x ADC and stores calibration data.

### mcp356x_calibration_sbeam
Specialized calibration for S-beam load cells with filtering and multiple data processing paths.

## Telemetry Examples

### mcp356x_telemetry_12_cells
Reads data from 12 load cells and sends readings in a format compatible with the Telemetry Viewer application.

## 6-Axis Force/Torque Sensor Calibration

### mcp356x_calibration_6axis_forces
Complete example for calibrating and reading from a 6-axis force/torque sensor built with four 3-axis load cells.

### mcp356x_calibration_6axis_recorder
Interactive utility for recording and storing calibration data from a 6-axis force/torque sensor.

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
├── mcp356x_telemetry_calibration/
│   ├── mcp356x_calibration_6axis_forces.cpp
│   ├── mcp356x_calibration_6axis_recorder.cpp
│   ├── TelemetryConfiguration.txt
│   └── TelemetryConfiguration - connection 0 - UART COM3.csv
├── mcp356x_calibration_all_channels.cpp
├── mcp356x_calibration_force_basic.cpp
└── mcp356x_calibration_sbeam.cpp
```

## Calibration Methods

The examples in this folder demonstrate various calibration methods:

1. **Single Value Scale Factor**: Simple multiplication factor for converting raw readings to force units.
2. **Linear Calibration**: Using a linear equation (y = ax + b) for calibration.
3. **Polynomial Calibration**: Using a polynomial equation (y = ax² + bx + c) for improved accuracy.
4. **Matrix Calibration**: Using calibration matrices for multi-axis load cells with cross-axis compensation.

## Usage Notes

1. For single load cells, start with the basic calibration examples.
2. For more complex setups, like 3-axis or 6-axis sensors, use the appropriate specialized examples.
3. The Python script included with the basic calibration example helps analyze and generate calibration constants.
4. Telemetry examples are designed to work with the Telemetry Viewer application for real-time visualization.

## Hardware Setup

The examples use the following default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13

For different ADCs:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC0 | 7 | 6 |
| ADC1 | 4 | 5 |
| ADC2 | 2 | 3 |
