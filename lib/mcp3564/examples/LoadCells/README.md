# MCP356x Load Cell Examples

Examples demonstrating the use of MCP356x ADCs with various load cell configurations.

## Example Files

### Basic Load Cell Reading
- **mcp356x_basic_force_reading.cpp**: Basic reading and filtering of force data from a single load cell
- **mcp356x_basic_scale_reading.cpp**: Simple example showing basic scale operation with the MCP356xScale class
- **mcp356x_filtered_force_measurement.cpp**: Load cell measurement with Butterworth filtering for noise reduction

### Multi-Axis Load Cells
- **mcp356x_basic_triaxial_reading.cpp**: Basic reading from a 3-axis load cell using MCP356x3axis
- **mcp356x_triaxial_continuous_reading.cpp**: Continuous sampling and filtering of 3-axis load cell data
- **mcp356x_3axis_example.cpp**: Complete example using multiple 3-axis load cells with polynomial calibration

### Advanced Configurations
- **mcp356x_four_triaxial_calibration.cpp**: Calibration and operation of four 3-axis load cells
- **mcp356x_four_triaxial_sensor_plate.cpp**: Force sensor plate implementation using four 3-axis load cells
- **mcp356x_6axis_cell_reading.cpp**: Implementation of a 6-axis force/torque sensor using four 3-axis load cells
- **mcp356x_multiple_load_cell_manager.cpp**: Demonstrates managing up to 12 load cells simultaneously
- **mcp356x_polynomial_vs_matrix_comparison.cpp**: Comparison of polynomial and matrix calibration methods

## Hardware Setup

Default pin configuration for single ADC:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 7
- IRQ: Pin 6

For multi-ADC examples:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC1 | 7 | 6 |
| ADC2 | 4 | 5 |
| ADC3 | 2 | 3 |

## Calibration Methods

The examples demonstrate several calibration approaches:

1. **Scale Factor**: Simple multiplication factor for load cell raw values
2. **Linear Calibration**: Linear equation (y = mx + b) for calibration
3. **Polynomial Calibration**: Second-order polynomial (y = ax² + bx + c) for improved accuracy
4. **Matrix Calibration**: 3x3 matrix for 3-axis load cells to compensate for cross-axis sensitivity

## Usage

1. Connect your load cells to MCP356x ADCs according to the pin definitions
2. For multi-axis examples, ensure proper mechanical mounting and wiring
3. Upload the desired example
4. Open serial monitor at 115200 baud to view load cell readings
