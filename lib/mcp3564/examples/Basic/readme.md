# MCP356x Basic Examples

Simple examples demonstrating fundamental operations of the MCP356x ADC family.

## Example Files

### basic_differential_reading.cpp
Reads a differential channel from the ADC and displays voltage and raw values.

### multiple_adc_sampling.cpp
Shows how to configure and read from multiple MCP356x ADCs simultaneously.

### multiple_channel_operations.cpp
Demonstrates scanning and reading multiple channels on a single ADC.

### register_operations.cpp
Provides access to register-level operations with an interactive command interface.

### sampling_rate_measurement.cpp
Measures ADC conversion time and calculates actual samples per second.

### single_channel_operations.cpp
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
| ADC0 | 7 | 6 |
| ADC1 | 4 | 5 |
| ADC2 | 2 | 3 |

## Usage

1. Connect your MCP356x ADC according to the pin definitions
2. Adjust pin definitions in the code if needed
3. Upload the example
4. Open serial monitor at 115200 baud to view results
