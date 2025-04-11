# MCP356x Load Cell Library

A comprehensive Arduino library for interfacing with MCP356x ADCs to manage and calibrate load cells, with special support for 3-axis and 6-axis force/torque sensing.

![MCP356x Library Architecture](https://via.placeholder.com/800x400?text=MCP356x+Library+Architecture)

## Features

- **Complete ADC Interface**
  - Support for MCP3561, MCP3562, and MCP3564 ADCs
  - High-resolution (24-bit) measurements
  - Fast sampling rates (up to 153.6 kSPS)
  - Multiple channel configurations

- **Load Cell Management**
  - Single-axis load cells (up to 12)
  - 3-axis load cells (up to 4)
  - 6-axis force/torque sensors
  - Multiple calibration methods

- **Signal Processing**
  - Digital filtering options
  - Noise reduction techniques
  - Averaging and tare functions
  - Matrix-based calibration

- **Advanced Applications**
  - Real-time force/torque sensing
  - Telemetry and data visualization
  - Multi-axis force platforms
  - Precision weighing systems

## Library Structure

```
MCP356x/
├── src/
│   ├── MCP356x.h            # Base ADC communication class
│   ├── MCP356x.cpp          # Implementation of base class
│   ├── MCP356xScale.h       # Load cell management class
│   ├── MCP356xScale.cpp     # Implementation of scale class
│   ├── MCP356x3axis.h       # 3-axis load cell class
│   ├── MCP356x3axis.cpp     # Implementation of 3-axis class
│   ├── MCP356x6axis.h       # 6-axis force/torque sensor class
│   ├── MCP356x6axis.cpp     # Implementation of 6-axis class
│   └── README.md            # Library documentation
│
├── examples/
│   ├── Basic/               # Fundamental ADC operations
│   ├── Calibration/         # Calibration techniques
│   ├── Comparison/          # Comparison with HX711
│   ├── Filtering/           # Noise reduction methods
│   ├── LoadCells/           # Load cell applications
│   └── Telemetry/           # Data streaming examples
│
├── include/                 # Additional header files
│   └── README.md            # Header documentation
│
├── keywords.txt             # Arduino IDE syntax highlighting
├── library.json             # PlatformIO configuration
├── library.properties       # Arduino Library Manager metadata
└── README.md                # This file
```

## Getting Started

### Installation

#### Arduino IDE
1. In Arduino IDE menu: Sketch > Include Library > Manage Libraries...
2. Search for "MCP356x"
3. Click Install

#### Manual Installation
1. Download this repository as a ZIP file
2. In Arduino IDE: Sketch > Include Library > Add .ZIP Library...
3. Select the downloaded ZIP file

#### PlatformIO
```
pio lib install "MCP356x"
```

### Basic Usage

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

## Example Guide

Start with these examples based on your application:

1. **New to ADCs**: [Basic Differential Reading](examples/Basic/mcp356x_basic_differential_reading.cpp)
2. **Weight Scales**: [Basic Force Reading](examples/LoadCells/mcp356x_basic_force_reading.cpp)
3. **Precision Measurement**: [Filtered Force Measurement](examples/LoadCells/mcp356x_filtered_force_measurement.cpp)
4. **Multi-Axis Sensing**: [3-Axis Example](examples/LoadCells/mcp356x_3axis_example.cpp)
5. **Force/Torque Sensing**: [6-Axis Cell Reading](examples/LoadCells/mcp356x_6axis_cell_reading.cpp)
6. **Calibration**: [Calibration Force Basic](examples/Calibration/mcp356x_calibration_force_basic.cpp)
7. **Real-time Visualization**: [6-Axis Calibrated Telemetry](examples/Telemetry/mcp356x_6axis_calibrated_telemetry.cpp)

## Hardware Requirements

- Arduino-compatible microcontroller (Teensy, ESP32, etc.)
- MCP3561, MCP3562, or MCP3564 ADC
- Load cell(s) compatible with your application
- Resistors, capacitors for proper ADC configuration
- Optional: Filtering capacitors for noise reduction


## Dependencies

- [BasicLinearAlgebra](https://github.com/tomstewart89/BasicLinearAlgebra) - For matrix operations
- Arduino SPI library (built-in)

## Contributing

Contributions to improve the library are welcome:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This library is released under the MIT License. See the `LICENSE` file for details.

## Acknowledgments

- Tom Stewart for the BasicLinearAlgebra library
- The Arduino community for their ongoing support

## Author

Jose Luis Berna Moya

## Version History

- 1.0.0 (March 2025): Initial release