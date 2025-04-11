# MCP356x Filtering Examples

Examples demonstrating various filtering techniques for MCP356x ADC readings.

## Example Files

### mcp356x_filtering_adc_readings.cpp
Demonstrates Butterworth filtering of ADC readings from MCP356x, with timing measurements and performance analysis.

### mcp356x_multiple_filter_comparison.cpp
Compares multiple filtering techniques (Butterworth, Moving Average, Median, Notch) on the same ADC data stream to evaluate their effectiveness.

## Hardware Setup

Default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 7
- IRQ: Pin 6

## Filter Types

The examples demonstrate several filtering approaches:

1. **Butterworth Filter**: A maximally flat magnitude filter for smooth frequency response
2. **Simple Moving Average (SMA)**: Averages the last N samples to reduce noise
3. **Median Filter**: Removes outliers by selecting the median value from a window of samples
4. **Notch Filter**: Removes a specific frequency (like 50/60Hz noise)
5. **Custom Running Average**: Efficient implementation of a moving average

## Usage

1. Connect your MCP356x ADC according to the pin definitions
2. Adjust filter parameters if needed
3. Upload the example
4. Open serial monitor at 115200 baud to view results
5. For filter comparison, you can capture the serial output and import it into a spreadsheet for visualization
