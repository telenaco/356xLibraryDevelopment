# MCP356x Comparison Examples

This folder contains examples that compare the MCP356x ADC with the commonly used HX711 ADC for load cell applications.

## Example Files

### mcp356x_comparison_hx711_sampling_rate.cpp
Measures the sampling rate of the HX711 ADC using interrupt-driven detection of new data. This provides a baseline for comparison with the MCP356x ADC.

### mcp356x_comparison_hx711_vs_mcp356x_force.cpp
Directly compares force readings from both MCP356x and HX711 ADCs connected to identical load cells. It shows raw readings and filtered readings from both ADCs side by side.

### mcp356x_comparison_hx711_vs_mcp356x_sampling.cpp
Compares the sampling performance of MCP356x and HX711 ADCs by measuring the time between successful readings for each. This demonstrates the higher throughput of the MCP356x ADC.

## Hardware Setup

For MCP356x ADC:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 2 or 7 (varies by example)
- IRQ: Pin 3 or 6 (varies by example)

For HX711 ADC:
- DOUT: Pin 23
- SCK: Pin 22

## Key Differences Between MCP356x and HX711

1. **Resolution**:
   - MCP356x: Up to 24-bit resolution
   - HX711: 24-bit resolution

2. **Sampling Rate**:
   - MCP356x: Up to 153.6 kSPS (variable with oversample ratio)
   - HX711: Fixed at 10 or 80 SPS (depending on mode)

3. **Multiple Channels**:
   - MCP356x: Up to 8 single-ended or 4 differential channels
   - HX711: 2 channels (one with gain 128, one with gain 32/64)

4. **Configurability**:
   - MCP356x: Highly configurable (gain, oversample ratio, etc.)
   - HX711: Limited configurations (gain selection by channel)

5. **Interface**:
   - MCP356x: Standard SPI interface
   - HX711: Custom two-wire interface

## Usage

Connect the ADCs according to the pin configurations in each example and upload the code to your microcontroller. Open the serial monitor at 115200 baud to view the comparison results.

For analysis of the results, you can capture the serial output and import it into a spreadsheet or data visualization tool.
