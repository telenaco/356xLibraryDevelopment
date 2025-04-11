# MCP356x Load Cell Library

A comprehensive Arduino library for interfacing with MCP356x ADCs to manage and calibrate load cells, with special support for 3-axis and 6-axis force/torque sensing.

## Overview

This library provides a complete solution for working with load cells connected to MCP356x ADCs. It is designed with a layered architecture:

- `MCP356x` - Low-level interface to the MCP356x ADC
- `MCP356xScale` - Mid-level interface for managing multiple load cells
- `MCP356x3axis` - High-level interface for 3-axis load cells
- `MCP356x6axis` - High-level interface for 6-axis force/torque sensors (using four 3-axis load cells)

## Features

- Support for multiple MCP356x ADCs (up to 3)
- Support for multiple load cells (up to 12)
- Multiple calibration methods:
  - Digital (raw readings)
  - Single value (scale factor)
  - Linear (ax + b)
  - Polynomial (ax² + bx + c)
- Matrix-based calibration for 3-axis and 6-axis sensors
- Tare functionality
- Axis inversion
- Conversion between grams-force and Newtons
- Force vector to quaternion conversion (for visualization)
- Comprehensive debug and print functions

## Hardware Setup

The library is designed to work with the following ADCs:
- MCP3561
- MCP3562
- MCP3564

Each ADC can support up to 4 differential channels, allowing you to connect:
- Up to 12 single-axis load cells
- Up to 4 3-axis load cells
- One 6-axis force/torque sensor (using four 3-axis load cells)

### Pin Connections

For each MCP356x ADC:
- SDI: Serial Data Input
- SDO: Serial Data Output
- SCK: Serial Clock Input
- CS: Chip Select
- IRQ: Interrupt Request (for data-ready signaling)

## Class Documentation

### MCP356x

This is the base class that handles direct communication with the MCP356x ADC via SPI.

```cpp
MCP356x(const MCP356xConfig& config);
```

Key methods:
- `int8_t init(SPIClass*)` - Initialize the ADC
- `int updatedReadings()` - Check for and process updated ADC readings
- `int32_t value(MCP356xChannel)` - Get the value for a specific channel
- `double valueAsVoltage(MCP356xChannel)` - Get the voltage for a specific channel
- `void printRegs(StringBuilder*)` - Print register contents
- `void printData(StringBuilder*)` - Print ADC data and status

### MCP356xScale

This class manages multiple load cells connected to one or more MCP356x ADCs.

```cpp
MCP356xScale(int totalScales, int sckPin, int sdoPin, int sdiPin,
             int irqPin1, int csPin1, int irqPin2 = -1, int csPin2 = -1, 
             int irqPin3 = -1, int csPin3 = -1);
```

Key methods:
- `int updatedAdcReadings()` - Update ADC readings
- `void setDigitalRead(int scaleIndex)` - Configure digital reading mode
- `void setScaleFactor(int scaleIndex, float scale)` - Set simple scale factor
- `void setLinearCalibration(int scaleIndex, float slope, float intercept)` - Set linear calibration
- `void setPolynomialCalibration(int scaleIndex, float a, float b, float c)` - Set polynomial calibration
- `void tare(int scaleIndex, int times = 1000)` - Tare the scale
- `float getReading(int scaleIndex)` - Get calibrated reading
- `float getForce(int scaleIndex)` - Get force in Newtons

### MCP356x3axis

This class provides functionality for 3-axis load cells.

```cpp
MCP356x3axis(MCP356xScale* scale, int xIndex, int yIndex, int zIndex);
```

Key methods:
- `void setCalibrationMatrix(const Matrix<3, 3>& calibMatrix)` - Set matrix calibration
- `void setCalibrationPolynomial(const PolynomialCoefficients& xCoeffs, const PolynomialCoefficients& yCoeffs, const PolynomialCoefficients& zCoeffs)` - Set polynomial calibration
- `void setAxisInversion(bool invertX, bool invertY, bool invertZ)` - Set axis inversion
- `void tare(int numReadings = 100)` - Tare the load cell
- `Matrix<3, 1> getDigitalReading()` - Get raw digital readings
- `Matrix<3, 1> getGfReading()` - Get calibrated readings in grams-force
- `Matrix<3, 1> getNewtonReading()` - Get readings in Newtons
- `Quaternion forceVectorToQuaternion(Matrix<3, 1> forceReading)` - Convert force to quaternion

### MCP356x6axis

This class implements a 6-axis force/torque sensor using four 3-axis load cells.

```cpp
MCP356x6axis(MCP356x3axis* loadCellA, MCP356x3axis* loadCellB, MCP356x3axis* loadCellC, MCP356x3axis* loadCellD, float plateWidth, float plateLength);
```

Key methods:
- `void setCalibrationMatrix(const BLA::Matrix<6, 6>& calibMatrix)` - Set 6-axis calibration matrix
- `BLA::Matrix<6, 1> readRawForceAndTorque()` - Get raw force and torque readings
- `BLA::Matrix<6, 1> readCalibratedForceAndTorque()` - Get calibrated force and torque readings
- `void tare()` - Tare all load cells
- `void reset()` - Reset all calibration data

## Examples

### Basic Load Cell Reading

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
  while (!Serial) {
    delay(10);
  }

  // Initialize with 1 scale
  mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);
  
  // Set calibration factor (grams per ADC unit)
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

### 3-Axis Load Cell Example

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

### 6-Axis Force/Torque Sensor Example

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
  
  // Set calibration for each load cell
  // ... (calibration code here)
  
  // Initialize the 6-axis force/torque sensor
  float plateWidth = 0.125;  // plate width in meters
  float plateLength = 0.125; // plate length in meters
  sixAxisLoadCell = new MCP356x6axis(loadCell1, loadCell2, loadCell3, loadCell4, 
                                    plateWidth, plateLength);
  
  // Set the 6-axis calibration matrix
  // ... (calibration code here)
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

## Dependencies

- [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) - For matrix operations
- Arduino SPI library

## License

This library is released under the MIT License.

## Author

Jose Luis Berna Moya

## Version History

- 1.0.0 (March 2025): Initial release
