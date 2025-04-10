# MCP356x Basic Examples

Simple examples demonstrating fundamental operations of the MCP356x ADC family.

## Example Files

### mcp356x_basic_differential_reading.cpp
Reads a differential channel from the ADC and displays voltage and raw values every 5 seconds.

### mcp356x_multiple_adc_sampling.cpp
Shows how to configure and read from multiple MCP356x ADCs simultaneously, with sampling rate measurement.

### mcp356x_multiple_channel_operations.cpp
Demonstrates scanning and reading multiple channels on multiple ADCs.

### mcp356x_register_operations.cpp
Provides access to register-level operations with an interactive command interface.

### mcp356x_sampling_rate_measurement.cpp
Measures ADC conversion time and calculates actual samples per second.

### mcp356x_single_channel_operations.cpp
Minimal example focused on reading a single ADC channel continuously.

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

## Usage

1. Connect your MCP356x ADC according to the pin definitions
2. Adjust pin definitions in the code if needed
3. Upload the example
4. Open serial monitor at 115200 baud to view results
