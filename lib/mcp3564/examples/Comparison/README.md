# MCP356x Comparison Examples

This folder contains examples that compare the MCP356x ADC with the widely used HX711 ADC for load cell and precision weight measurement applications.

## Purpose

These comparison examples enable you to:
- Evaluate the performance differences between MCP356x and HX711 ADCs
- Compare sampling rates, resolution, and noise characteristics
- Make informed decisions about which ADC is best for your application
- Understand the advantages and tradeoffs of each ADC

## Example Files

### mcp356x_comparison_hx711_sampling_rate.cpp
Measures the actual sampling rate of the HX711 ADC using interrupt-driven detection of new data.
- Calculates precise timing between readings
- Reports exact sampling frequency in Hz
- Evaluates stability of sampling intervals
- Baseline measurement for comparison with MCP356x

### mcp356x_comparison_hx711_vs_mcp356x_force.cpp
Directly compares force readings from both ADCs connected to identical load cells.
- Side-by-side comparison of raw readings
- Application of Butterworth filtering to both data streams
- Analysis of noise levels and stability
- Performance under identical conditions

### mcp356x_comparison_hx711_vs_mcp356x_sampling.cpp
Comprehensive comparison of sampling performance between the two ADCs.
- Measures time between successful readings for each ADC
- Outputs timestamps, raw readings, and status flags
- Demonstrates the higher throughput capability of MCP356x
- Evaluates real-world performance differences

## Hardware Setup

### Required Components
- Arduino-compatible microcontroller
- MCP356x ADC (MCP3561, MCP3562, or MCP3564)
- HX711 ADC module
- 1-4 load cells (depending on example)
- Breadboard and jumper wires

### Connection Diagram for MCP356x
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 2 or 7 (varies by example)
- IRQ: Pin 3 or 6 (varies by example)

### Connection Diagram for HX711
- DOUT: Pin 23
- SCK: Pin 22

## Key Performance Differences

| Feature | MCP356x | HX711 | Advantage |
|---------|---------|-------|-----------|
| **Resolution** | 24-bit | 24-bit | Equal |
| **Sampling Rate** | Up to 153.6 kSPS | 10/80 SPS | MCP356x (19-1500x faster) |
| **Channels** | Up to 8 SE / 4 DIFF | 2 channels | MCP356x |
| **Configurability** | Highly configurable | Limited | MCP356x |
| **Interface** | Standard SPI | Custom 2-wire | MCP356x (easier integration) |
| **Power Consumption** | 180µA - 2.7mA | 1.5mA | Application dependent |
| **Cost** | Higher | Lower | HX711 |
| **Complexity** | More complex | Simpler | HX711 (easier to start with) |

## Analysis Methodology

The comparison examples follow this methodology:
1. Configure both ADCs for optimal performance
2. Apply identical load/weight to both sensors
3. Record data simultaneously from both ADCs
4. Apply identical filtering when applicable
5. Analyze timing, accuracy, and noise characteristics

## Data Analysis

For best results analyzing the comparison data:
1. Capture the serial output to a file
2. Import the data into a spreadsheet or data visualization tool
3. Create charts comparing the two ADCs' performance
4. Calculate statistics such as:
   - Standard deviation (noise measurement)
   - Sampling frequency stability
   - Response time to load changes

## When to Choose Each ADC

**Choose MCP356x when:**
- You need high sampling rates (>100 samples/second)
- Multiple load cells must be read through a single ADC
- You require advanced filtering and configuration options
- SPI is already used in your project

**Choose HX711 when:**
- Lower cost is a priority
- Simpler integration is desired
- Standard load cell sampling rates are sufficient
- You're new to load cell interfacing

## Getting Started

1. Connect both ADCs according to the pin configurations in each example
2. Upload the desired comparison code to your microcontroller
3. Open the serial monitor at 115200 baud
4. Apply test weights or forces to your load cells
5. Capture the data for analysis