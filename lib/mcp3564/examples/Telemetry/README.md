# MCP356x Telemetry Examples

This folder contains examples for streaming load cell data from MCP356x ADCs to visualization and telemetry software, enabling real-time monitoring and analysis of force data.

## Purpose

The telemetry examples demonstrate how to:
- Stream formatted data from MCP356x ADCs
- Configure high-speed data acquisition for telemetry
- Integrate with visualization software
- Implement real-time monitoring systems
- Control acquisition parameters via serial commands

## Example Files

### mcp356x_telemetry_12_cells.cpp
Streams data from 12 load cells with a telemetry-compatible format:
- Reads data from 12 channels (up to three MCP356x ADCs)
- Formats data for Telemetry Viewer application
- Calculates rolling averages for stable readings
- Responds to tare commands via serial interface
- Compatible with included Telemetry Viewer configuration

### mcp356x_triaxial_telemetry.cpp
Streams data from four 3-axis load cells with orientation information:
- Outputs force readings from all axes (12 channels total)
- Includes quaternion orientation data for visualization
- Supports configurable sample intervals
- Formats data for compatibility with visualization tools
- Includes axis inversion for sensor orientation correction

### mcp356x_6axis_telemetry.cpp
Implements basic 6-axis force/torque sensor telemetry:
- Combines four 3-axis load cells into a 6-axis sensor
- Outputs combined forces (Fx, Fy, Fz) and torques (Mx, My, Mz)
- Includes timing information for performance analysis
- Structured for real-time visualization
- Demonstrates sensor fusion approach

### mcp356x_6axis_calibrated_telemetry.cpp
Enhanced version with calibration matrix for 6-axis sensors:
- Applies full 6×6 calibration matrix to force/torque readings
- High accuracy measurements for all six degrees of freedom
- Averaging for noise reduction
- Supports tare commands for zeroing the sensor
- Optimized for precision measurements

### mcp356x_6axis_fast_update.cpp
High-speed version with motor control for actuator testing:
- Optimized for maximum update rates
- Integrates motor control with force/torque sensing
- Implements PWM ramp patterns for automated testing
- Synchronized data acquisition and control
- Ideal for dynamic response testing and characterization

## Hardware Setup

### Pin Configuration
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13

MCP356x ADC connections:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC0 | 7 | 6 |
| ADC1 | 4 | 5 |
| ADC2 | 2 | 3 |

For motor control in fast_update example:
- MOTOR_PWM_PIN: Pin 9

## Data Format

### 12-Cell Telemetry
```
cell1,cell2,cell3,cell4,cell5,cell6,cell7,cell8,cell9,cell10,cell11,cell12
```

### 3-Axis Telemetry
```
x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4,qw1,qx1,qy1,qz1,qw2,qx2,qy2,qz2,qw3,qx3,qy3,qz3,qw4,qx4,qy4,qz4
```

### 6-Axis Telemetry
```
timestamp,Fx,Fy,Fz,Mx,My,Mz
```

## Telemetry Viewer Configuration

These examples are designed to work with the Telemetry Viewer application, which provides real-time visualization of the streaming data.

To use with Telemetry Viewer:
1. Upload the example code to your microcontroller
2. Connect via USB (virtual serial port)
3. Open Telemetry Viewer
4. Load the corresponding configuration file from the example folder
5. Connect to the COM port at 115200 baud
6. View real-time data visualization

## Serial Commands

The examples support these serial commands:

| Command | Description | Example |
|---------|-------------|---------|
| `T,1000` | Perform tare with 1000 samples | `T,1000` |
| `R` | Reset calibration | `R` |
| `P` | Print current configuration | `P` |

## Visualization Options

1. **Telemetry Viewer**
   - Configured with included settings files
   - Real-time charts and gauges
   - Data logging capabilities
   - Multi-channel visualization

2. **Processing**
   - 3D visualization libraries
   - Custom visualization scripts
   - Interactive force display

3. **Excel or MATLAB**
   - Data capture via serial monitor
   - Advanced analysis and plotting
   - Post-processing of data

## Sample Rate Considerations

When configuring telemetry systems, consider these sample rate limitations:

| Configuration | Max Sample Rate | Notes |
|---------------|-----------------|-------|
| 12 channels   | ~400 Hz         | Limited by serial bandwidth |
| 6-axis basic  | ~800 Hz         | Requires proper averaging |
| 6-axis fast   | ~1200 Hz        | Optimized for speed |

## Getting Started

1. Select the appropriate example for your sensor configuration
2. Connect hardware according to pin definitions (modify if needed)
3. Upload the example to your microcontroller
4. Open the visualization tool of your choice
5. Configure to match the data format of the example
6. For Telemetry Viewer, load the included configuration file
7. Start monitoring your force/torque data in real-time

## Application Ideas

- Force-feedback control systems
- Material testing equipment
- Robotic touch/contact sensing
- Industrial process monitoring
- Sports equipment analysis
- Biomechanical research
- Physical therapy tools