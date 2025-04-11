# MCP356x Load Cell Examples

This folder contains examples demonstrating the use of MCP356x ADCs with various load cell configurations, from simple single-axis scales to complex multi-axis force/torque sensors.

## Purpose

The load cell examples demonstrate how to:
- Interface with different types of load cells using MCP356x ADCs
- Configure and calibrate for accurate weight and force measurements
- Implement multi-axis force sensing systems
- Create complete weighing solutions for practical applications

## Example Files

### Basic Load Cell Reading

#### mcp356x_basic_force_reading.cpp
Demonstrates simple load cell interface with filtering:
- Basic reading and filtering of force data
- Tare functionality for zeroing the scale
- Performance metrics and timing information
- Butterworth filtering for noise reduction

#### mcp356x_basic_scale_reading.cpp
Simple example showing basic scale operation:
- Raw ADC reading and conversion to weight
- Minimal implementation for beginners
- Single channel operation with the MCP356xScale class
- Simple output formatting

#### mcp356x_filtered_force_measurement.cpp
Enhanced load cell reading with advanced filtering:
- Optimized Butterworth filtering parameters
- Real-time display of both raw and filtered readings
- Adjustable filter characteristics
- Performance optimized for stable readings

### Multi-Axis Load Cells

#### mcp356x_basic_triaxial_reading.cpp
Introduction to 3-axis load cell reading:
- Reading from X, Y, and Z axes simultaneously
- Basic data acquisition from a 3-axis load cell
- Simple tare operations on all axes
- Digital readings without advanced calibration

#### mcp356x_triaxial_continuous_reading.cpp
Continuous sampling from 3-axis load cells:
- High-speed continuous sampling
- Real-time filtering for each axis
- CSV output format for data logging
- Long-term stability for monitoring applications

#### mcp356x_3axis_example.cpp
Complete example for multiple 3-axis load cells:
- Configuration for up to four 3-axis load cells
- Polynomial calibration for each axis
- Tare operations on individual cells
- Synchronized reading from multiple cells

### Advanced Configurations

#### mcp356x_four_triaxial_calibration.cpp
Calibration of multiple 3-axis load cells:
- Matrix-based calibration for four 3-axis load cells
- Cross-axis compensation for accurate readings
- Synchronized reading from 12 ADC channels
- Formatted output of calibrated readings

#### mcp356x_four_triaxial_sensor_plate.cpp
Force sensor plate using multiple 3-axis load cells:
- Combines readings from four 3-axis load cells
- Quaternion representation for force direction
- Advanced signal processing for plates and platforms
- Complete force sensor plate implementation

#### mcp356x_6axis_cell_reading.cpp
6-axis force/torque sensor implementation:
- Three-dimensional force (Fx, Fy, Fz) readings
- Three-dimensional torque (Mx, My, Mz) readings
- Calibration for cross-coupling between axes
- Complete 6-DOF (degree of freedom) force sensing

#### mcp356x_multiple_load_cell_manager.cpp
Management of multiple load cells:
- Handling up to 12 load cells simultaneously
- Different calibration methods for each cell
- Channel mapping across multiple ADCs
- Comprehensive load cell management solution

#### mcp356x_polynomial_vs_matrix_comparison.cpp
Comparison of calibration methods:
- Side-by-side comparison of polynomial and matrix calibration
- Performance analysis of different calibration techniques
- Debug information for calibration validation
- Guidance for choosing optimal calibration method

## Hardware Configurations

### Single Load Cell
```
Load Cell ---→ MCP356x ADC ---→ Microcontroller
```

### Multi-Channel Single ADC
```
Load Cell 1 ----+
Load Cell 2 ----+--→ MCP356x ADC ---→ Microcontroller
Load Cell 3 ----+
Load Cell 4 ----+
```

### 3-Axis Load Cell
```
        +--- X Channel ---+
3-Axis --+--- Y Channel ---+--→ MCP356x ADC ---→ Microcontroller
        +--- Z Channel ---+
```

### 6-Axis Force/Torque Sensor
```
3-Axis LC 1 ----+
3-Axis LC 2 ----+
                +--→ Three MCP356x ADCs ---→ Microcontroller
3-Axis LC 3 ----+
3-Axis LC 4 ----+
```

## Pin Configurations

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

The examples demonstrate these calibration approaches:

1. **Scale Factor Calibration**  
   Simple multiplication factor:
   ```cpp
   mcpScale->setScaleFactor(0, 0.0006f);
   ```

2. **Linear Calibration**  
   Linear equation (y = mx + b):
   ```cpp
   mcpScale->setLinearCalibration(0, 0.0006f, -538.5f);
   ```

3. **Polynomial Calibration**  
   Second-order polynomial (y = ax² + bx + c):
   ```cpp
   mcpScale->setPolynomialCalibration(0, 0.0f, 0.0006f, -538.5f);
   ```

4. **Matrix Calibration**  
   For 3-axis load cells with cross-axis compensation:
   ```cpp
   BLA::Matrix<3, 3> calibMatrix = {
     5.98e-04, -1.07e-05, 7.76e-06,
     2.59e-06, 5.87e-04, 6.22e-06,
     5.95e-06, 1.36e-05, 5.98e-04
   };
   loadCell->setCalibrationMatrix(calibMatrix);
   ```

## Axis Inversion

For proper orientation of multi-axis sensors:
```cpp
// Configure axis directions (1 = invert, 0 = normal)
loadCell1->setAxisInversion(1, 1, 0);
loadCell2->setAxisInversion(0, 1, 1);
loadCell3->setAxisInversion(0, 0, 0);
loadCell4->setAxisInversion(1, 1, 1);
```

## Getting Started

1. Start with `mcp356x_basic_force_reading.cpp` for simple weight measurement
2. For multi-axis sensors, use the appropriate example based on your load cell type
3. Connect hardware according to the pin definitions (modify in code if needed)
4. Upload the example to your microcontroller
5. Open serial monitor at 115200 baud to view readings
6. Follow calibration procedures for accurate measurements

## Application Tips

1. **For Weight Scales:**
   - Use `mcp356x_basic_force_reading.cpp` with filtering
   - Perform careful calibration with known weights
   - Implement temperature compensation for drift correction

2. **For 3D Force Sensing:**
   - Use `mcp356x_triaxial_continuous_reading.cpp`
   - Mount load cells securely to avoid mechanical noise
   - Calibrate all axes for cross-axis interference

3. **For Force/Torque Sensors:**
   - Use `mcp356x_6axis_cell_reading.cpp`
   - Build a rigid mechanical structure
   - Perform full 6-axis calibration routine