# MCP356x Basic Examples

These examples demonstrate fundamental operations of the MCP356x ADC family, providing a foundation for more complex applications.

## Purpose

The basic examples are designed to help you:
- Understand core functionality of the MCP356x ADCs
- Learn how to initialize and configure the ADC
- Read data from single and multiple channels
- Work with multiple ADC devices simultaneously
- Measure performance characteristics

## Example Files

### mcp356x_basic_differential_reading.cpp
This example demonstrates reading a differential channel from the MCP356x ADC. It shows how to:
- Initialize SPI communication with the ADC
- Configure a single differential channel
- Read voltage and raw ADC values
- Display timing information

### mcp356x_single_channel_operations.cpp
A minimal example focused on reading a single ADC channel continuously. Perfect for:
- Getting started with the library
- Simple data acquisition tasks
- Understanding the basic read loop structure

### mcp356x_multiple_channel_operations.cpp
Demonstrates how to scan and read multiple channels. This example shows:
- How to configure channel scanning
- Reading values from all differential channels
- Handling multiple data streams
- Measuring performance metrics

### mcp356x_multiple_adc_sampling.cpp
Works with multiple MCP356x ADCs simultaneously. Features:
- Configuring multiple ADCs with different chip select pins
- Coordinating readings across devices
- Calculating combined sampling rates
- Managing multiple interrupt sources

### mcp356x_register_operations.cpp
Provides an interactive interface for exploring the MCP356x registers. Includes:
- A terminal-based menu system
- Direct register reading and writing
- Changing configuration parameters in real-time
- Detailed debug output

### mcp356x_sampling_rate_measurement.cpp
Measures the actual achievable sampling rate with different configurations:
- Tests different oversampling ratios
- Measures conversion times
- Calculates samples per second
- Shows timing relationships

## Hardware Setup

Default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 7
- IRQ: Pin 6

For multi-ADC examples:
| ADC | CS Pin | IRQ Pin |
|-----|--------|---------|
| ADC0/1 | 7 | 6 |
| ADC1/2 | 4 | 5 |
| ADC2/3 | 2 | 3 |

## Getting Started

1. Connect your MCP356x ADC according to the pin definitions (modify in code if needed)
2. Choose the example that best fits your learning goals
3. Upload the example to your microcontroller
4. Open the serial monitor at 115200 baud to view results

## Next Steps

After mastering these basic examples, explore:
- [Calibration Examples](../Calibration/): For accurate measurements
- [Filtering Examples](../Filtering/): For noise reduction 
- [Load Cell Examples](../LoadCells/): For weight sensing applications