# MCP356x Include Directory

This directory contains the header files for the MCP356x library. These headers define the interfaces for the various classes and structures used throughout the library.

## Purpose

Header files serve several important functions in this library:
- They define the class interfaces and public methods
- They declare the data structures and enumerations
- They specify default parameters and constants
- They document the API for users of the library

## Header Files

The library's functionality is organized into several header files:

### MCP356x.h
Contains the base class for interfacing directly with MCP356x ADCs. This class provides:
- SPI communication with the ADC
- Register access and configuration
- Reading of ADC data
- Timing and interrupt handling

### MCP356xScale.h
Contains the class for managing multiple load cells. This class provides:
- Management of up to 12 load cells
- Support for multiple calibration methods
- Channel mapping across multiple ADCs
- Tare operations and conversion to force units

### MCP356x3axis.h
Contains the class for 3-axis load cell handling. This class provides:
- Matrix-based calibration for 3-axis sensors
- Axis inversion control
- Conversion between coordinate systems
- Quaternion representation for visualization

### MCP356x6axis.h
Contains the class for 6-axis force/torque sensors. This class provides:
- Management of four 3-axis load cells as a single 6-DOF sensor
- 6×6 calibration matrix support
- Force and torque vector calculation
- Plate geometry configuration

## Usage Guidelines

When using these headers in your code:

1. Include only the highest-level header you need:
   - For basic ADC access: `#include <MCP356x.h>`
   - For single-axis load cells: `#include <MCP356xScale.h>`
   - For 3-axis load cells: `#include <MCP356x3axis.h>`
   - For 6-axis sensors: `#include <MCP356x6axis.h>`

2. Each higher-level header automatically includes the headers it depends on, so you don't need to include them manually.

3. Review the class documentation in each header file for detailed API information.

4. Note the default values and constants defined in each header.