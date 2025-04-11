# MCP356x Telemetry Examples

Examples for streaming load cell data from MCP356x ADCs to telemetry visualization software.

## Example Files

### mcp356x_telemetry_12_cells.cpp
Reads data from 12 load cells and formats them for viewing in Telemetry Viewer application.

### mcp356x_triaxial_telemetry.cpp
Streams data from four 3-axis load cells (12 channels total) with quaternion orientation data.

### mcp356x_6axis_telemetry.cpp
Implements a 6-axis force/torque sensor telemetry stream using four 3-axis load cells.

### mcp356x_6axis_calibrated_telemetry.cpp
Enhanced version with calibration matrix applied to 6-axis force/torque readings.

### mcp356x_6axis_fast_update.cpp
Optimized version of the 6-axis sensor with faster update rates for real-time applications.

## Hardware Setup

Default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13

For the three ADCs:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC0 | 7 | 6 |
| ADC1 | 4 | 5 |
| ADC2 | 2 | 3 |

## Telemetry Viewer Configuration

These examples are designed to work with the Telemetry Viewer application, which can visualize the streaming data in real-time. Configuration files for Telemetry Viewer are included in the folder.

To use with Telemetry Viewer:
1. Upload the example code to your microcontroller
2. Connect via USB (virtual serial port)
3. Open Telemetry Viewer
4. Load the corresponding configuration file
5. Connect to the COM port at 115200 baud

## Usage Notes

- All examples stream CSV-formatted data via serial at 115200 baud
- Each example includes a timestamp for precise data logging
- Some examples support dynamic tare operations via serial commands (send "T,1000" to tare with 1000 samples)
- For 6-axis examples, data streams include:
  - Timestamp (microseconds)
  - Force values (Fx, Fy, Fz)
  - Torque values (Tx, Ty, Tz)
  - Additional sensor or control data (varies by example)
