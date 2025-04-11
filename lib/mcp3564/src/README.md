# MCP356x Load Cell Library

A comprehensive Arduino library for interfacing with MCP356x ADCs to manage and calibrate load cells, with special support for 3-axis and 6-axis force/torque sensing.

## Overview

This library provides a complete solution for working with load cells connected to MCP356x ADCs. It is designed with a layered architecture:

- `MCP356x` - Low-level interface to the MCP356x ADC
- `MCP356xScale` - Mid-level interface for managing multiple load cells
- `MCP356x3axis` - High-level interface for 3-axis load cells
- `MCP356x6axis` - High-level interface for 6-axis force/torque sensors (using four 3-axis load cells)

## Features

- **Extensive ADC Support**
  - Compatible with MCP3561, MCP3562, and MCP3564 ADCs
  - Support for multiple MCP356x ADCs (up to 3)
  - Support for multiple load cells (up to 12)

- **Versatile Calibration Options**
  - Digital (raw readings)
  - Single value (scale factor)
  - Linear (ax + b)
  - Polynomial (ax² + bx + c)
  - Matrix-based calibration for 3-axis and 6-axis sensors

- **Comprehensive Data Processing**
  - Tare functionality
  - Axis inversion
  - Conversion between grams-force and Newtons
  - Force vector to quaternion conversion (for visualization)
  - Integration with filtering libraries

- **Advanced Features**
  - Support for differential and single-ended measurements
  - Temperature sensing with the internal temperature sensor
  - Detailed debug and diagnostic functions
  - Performance optimization options

## Installation

1. Download the library as a ZIP file from the repository
2. In Arduino IDE: Sketch > Include Library > Add .ZIP Library...
3. Select the downloaded ZIP file
4. The library will be installed and available under "Contributed Libraries"

## Hardware Setup

The library is designed to work with:
- MCP3561 (2 differential channels)
- MCP3562 (4 differential channels)
- MCP3564 (8 differential channels or 4 differential + 8 single-ended)

### Pin Connections

For each MCP356x ADC:
- SDI: Serial Data Input (connect to MOSI pin)
- SDO: Serial Data Output (connect to MISO pin)
- SCK: Serial Clock Input
- CS: Chip Select
- IRQ: Interrupt Request (for data-ready signaling)

### Multiple ADC Setup

For applications requiring more than 4 channels:

```
MCU             ADC 1             ADC 2             ADC 3
---------------------------------------------------------------------
MOSI ---------> SDI -------------> SDI -------------> SDI
MISO <--------- SDO <------------- SDO <------------- SDO
SCK ----------> SCK -------------> SCK -------------> SCK
GPIO 1 --------> CS
GPIO 2 <-------- IRQ
GPIO 3 -----------------------> CS
GPIO 4 <----------------------- IRQ
GPIO 5 ----------------------------------> CS
GPIO 6 <---------------------------------- IRQ
```

## Library Architecture

### Class Hierarchy

```
MCP356x (Base ADC Class)
    |
    ├── MCP356xScale (Load Cell Manager)
    |       |
    |       └── MCP356x3axis (3-Axis Load Cell)
    |               |
    |               └── MCP356x6axis (6-Axis Force/Torque Sensor)
    |
    └── (Other future extensions)
```

### Memory Requirements

| Class          | Flash Usage | RAM Usage |
|----------------|-------------|-----------|
| MCP356x        | ~4 KB       | ~150 bytes |
| MCP356xScale   | ~2 KB       | ~450 bytes + 40 bytes per scale |
| MCP356x3axis   | ~3 KB       | ~160 bytes |
| MCP356x6axis   | ~2 KB       | ~180 bytes |

## Basic Usage

### Single Load Cell

```cpp
#include <MCP356xScale.h>

// Pin definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN  7
#define IRQ_PIN 6

MCP356xScale* mcpScale = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize with 1 scale
  mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);
  
  // Set calibration factor
  mcpScale->setScaleFactor(0, 0.0006f);
  
  // Tare the scale
  mcpScale->tare(0);
}

void loop() {
  if (mcpScale->updatedAdcReadings()) {
    float reading = mcpScale->getReading(0);
    Serial.print("Weight: ");
    Serial.print(reading);
    Serial.println(" g");
  }
}
```

### 3-Axis Load Cell

```cpp
#include <MCP356x3axis.h>

// Pin definitions
#define SDI_PIN  11
#define SDO_PIN  12
#define SCK_PIN  13
#define CS_PIN0  7
#define IRQ_PIN0 6

#define TOTAL_NUM_CELLS 3

MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize the scale
  mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0);
  
  // Initialize the 3-axis load cell with indices for X, Y, Z
  loadCell = new MCP356x3axis(mcpScale, 0, 1, 2);
  
  // Set calibration matrix
  Matrix<3, 3> calibMatrix = {
    5.98e-04, -1.07e-05, 7.76e-06,
    2.59e-06, 5.87e-04, 6.22e-06,
    5.95e-06, 1.36e-05, 5.98e-04
  };
  loadCell->setCalibrationMatrix(calibMatrix);
  
  // Tare the load cell
  loadCell->tare(100);
}

void loop() {
  mcpScale->updatedAdcReadings();
  
  Matrix<3, 1> gfReading = loadCell->getGfReading();
  
  Serial.print("X: ");
  Serial.print(gfReading(0));
  Serial.print(" g, Y: ");
  Serial.print(gfReading(1));
  Serial.print(" g, Z: ");
  Serial.print(gfReading(2));
  Serial.println(" g");
  
  delay(100);
}
```

### 6-Axis Force/Torque Sensor

```cpp
#include <MCP356x6axis.h>

// Pin definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN0 7
#define IRQ_PIN0 6
#define CS_PIN1 4
#define IRQ_PIN1 5
#define CS_PIN2 2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12

MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;
MCP356x6axis* sixAxisLoadCell = nullptr;

void setup() {
  Serial.begin(115200);
  
  // Initialize the scale with 12 channels
  mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, 
                             IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);
  
  // Initialize four 3-axis load cells
  loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);
  loadCell2 = new MCP356x3axis(mcpScale, 3, 4, 5);
  loadCell3 = new MCP356x3axis(mcpScale, 6, 7, 8);
  loadCell4 = new MCP356x3axis(mcpScale, 9, 10, 11);
  
  // Initialize the 6-axis force/torque sensor
  float plateWidth = 0.125;  // plate width in meters
  float plateLength = 0.125; // plate length in meters
  sixAxisLoadCell = new MCP356x6axis(loadCell1, loadCell2, loadCell3, loadCell4, 
                                    plateWidth, plateLength);
  
  // Set calibration and perform tare
  // (calibration matrix code would go here)
}

void loop() {
  if (mcpScale->updatedAdcReadings()) {
    // Read calibrated force and torque values
    BLA::Matrix<6, 1> forceAndTorque = sixAxisLoadCell->readCalibratedForceAndTorque();
    
    // Print the values
    Serial.print("Fx: ");
    Serial.print(forceAndTorque(0));
    Serial.print(" N, Fy: ");
    Serial.print(forceAndTorque(1));
    Serial.print(" N, Fz: ");
    Serial.print(forceAndTorque(2));
    Serial.print(" N, Mx: ");
    Serial.print(forceAndTorque(3));
    Serial.print(" Nm, My: ");
    Serial.print(forceAndTorque(4));
    Serial.print(" Nm, Mz: ");
    Serial.print(forceAndTorque(5));
    Serial.println(" Nm");
  }
}
```

## Example Folders

The library includes multiple example folders for different applications:

- **Basic**: Simple examples showing fundamental ADC operations
- **Calibration**: Examples demonstrating different calibration techniques
- **Comparison**: Benchmarks comparing MCP356x with HX711 ADCs
- **Filtering**: Examples showing noise reduction techniques
- **LoadCells**: Complete load cell interface examples
- **Telemetry**: Data streaming for visualization and analysis

## Dependencies

- [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) - For matrix operations
- Arduino SPI library

## Troubleshooting

### Common Issues

1. **No readings or zero values**
   - Check SPI connections
   - Verify CS and IRQ pin configurations
   - Ensure load cell is properly connected
   - Check calibration factors

2. **Noisy readings**
   - Add capacitors to power lines
   - Implement filtering (see Filtering examples)
   - Check for mechanical vibrations
   - Ensure stable power supply

3. **Drift over time**
   - Implement temperature compensation
   - Use auto-tare for long-term operation
   - Check for ADC reference voltage stability

### Diagnostic Functions

The library includes diagnostic functions for troubleshooting:

```cpp
// Print detailed ADC information
mcpScale->printADCsDebug();

// Print raw channel values
mcpScale->printAdcRawChannels();

// Print reading rate statistics
mcpScale->printReadsPerSecond();

// Print calibration parameters for all channels
mcpScale->printChannelParameters();
```

## License

This library is released under the MIT License.

## Author

Jose Luis Berna Moya

## Version History

- 1.0.0 (March 2025): Initial release