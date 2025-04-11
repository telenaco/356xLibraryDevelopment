# MCP356x Filtering Examples

These examples demonstrate various digital filtering techniques for improving the quality of MCP356x ADC readings, especially for load cell and precision measurement applications.

## Purpose

Noise is an inevitable challenge in precision measurement systems. These examples show how to:
- Apply different digital filters to ADC readings
- Compare filter performance and characteristics
- Choose the optimal filter for your application
- Implement real-time filtering techniques

## Example Files

### mcp356x_filtering_adc_readings.cpp
Demonstrates Butterworth filtering of MCP356x ADC readings.
- Implements a low-pass Butterworth filter with configurable parameters
- Measures and reports timing performance
- Outputs both raw and filtered readings
- Shows complete implementation with performance analysis

### mcp356x_multiple_filter_comparison.cpp
Compares multiple filtering techniques on the same ADC data stream.
- Implements and compares 5 different filter types:
  - Butterworth Filter
  - Simple Moving Average (SMA) 
  - Median Filter
  - Notch Filter
  - Custom Running Average
- Outputs performance metrics for each filter
- All filters process the same input for direct comparison
- CSV-formatted output for easy visualization

## Filter Types Explained

### Butterworth Filter
A maximally flat magnitude filter providing smooth frequency response with minimal passband ripple.
```cpp
auto butterworthFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
float filteredValue = butterworthFilter(rawReading);
```
- **Advantages**: Excellent frequency response, minimal distortion
- **Best for**: Removing high-frequency noise while preserving signal integrity

### Simple Moving Average (SMA)
Averages the last N samples to reduce random noise.
```cpp
SMA<50, int32_t, int32_t> simpleMovingAvg = {0};
int averagedValue = simpleMovingAvg(rawReading);
```
- **Advantages**: Easy to understand, computationally efficient
- **Best for**: Reducing white noise in relatively stable signals

### Median Filter
Selects the median value from a window of samples, excellent for removing outliers.
```cpp
MedianFilter<50, int32_t> medianFilter = {0};
int medianFilteredValue = medianFilter(rawReading);
```
- **Advantages**: Removes spikes and outliers without affecting other values
- **Best for**: Environments with occasional interference or sudden spikes

### Notch Filter
Removes a specific frequency while leaving other frequencies largely unaffected.
```cpp
auto notchFilter = simpleNotchFIR(NOTCH_FREQUENCY);
float notchedValue = notchFilter(rawReading);
```
- **Advantages**: Targets specific noise frequencies (e.g., 50/60Hz power line interference)
- **Best for**: Systems with known noise sources at specific frequencies

### Custom Running Average
Efficient implementation of a moving average using a circular buffer.
```cpp
int customRunningAverage = computeRunningAverage(rawReading);
```
- **Advantages**: Memory efficient, can be optimized for specific applications
- **Best for**: Resource-constrained environments requiring noise reduction

## Filter Performance Comparison

| Filter Type | CPU Usage | Memory Usage | Group Delay | Noise Reduction | Spike Rejection |
|-------------|-----------|--------------|-------------|-----------------|-----------------|
| Butterworth | Medium    | Low          | Medium      | Excellent       | Good            |
| SMA         | Low       | Medium       | High        | Good            | Poor            |
| Median      | High      | High         | Medium      | Good            | Excellent       |
| Notch       | Medium    | Low          | Low         | Targeted        | Poor            |
| Running Avg | Low       | Medium       | High        | Good            | Poor            |

## Hardware Setup

Default pin configuration:
- SDI (MOSI): Pin 11
- SDO (MISO): Pin 12
- SCK: Pin 13
- CS: Pin 7
- IRQ: Pin 6

## Key Filter Parameters

When using these examples, consider adjusting these key parameters:

### Butterworth Filter
```cpp
constexpr double SAMPLING_FREQUENCY = 11111;  // Your actual ADC sampling rate
constexpr double CUT_OFF_FREQUENCY = 2;       // Adjust based on signal characteristics
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
```

### SMA and Median Filters
```cpp
// Increase window size for more smoothing, decrease for faster response
SMA<50, int32_t, int32_t> simpleMovingAvg = {0};
MedianFilter<50, int32_t> medianFilter = {0};
```

### Notch Filter
```cpp
// Set to the frequency you want to remove (e.g., 50Hz for power line)
constexpr double NOTCH_FREQUENCY = 50;
```

## Filter Selection Guide

1. **For general noise reduction:**
   - Start with Butterworth filter with cutoff at ~10% of your signal bandwidth

2. **For eliminating random spikes:**
   - Use Median filter with window size based on spike duration

3. **For power line interference:**
   - Use Notch filter centered at 50Hz or 60Hz

4. **For resource-constrained systems:**
   - Use Simple Moving Average or Custom Running Average

5. **For optimal results:**
   - Consider cascading filters (e.g., Median → Butterworth)

## Data Visualization

To visualize filter performance:
1. Capture the serial output to a CSV file
2. Import into spreadsheet software or data visualization tool
3. Plot raw and filtered signals on the same chart
4. Analyze noise reduction and response characteristics

## Getting Started

1. Connect your MCP356x ADC according to the pin definitions
2. Adjust filter parameters to match your application needs
3. Upload the example to your microcontroller
4. Open serial monitor at 115200 baud to view results
5. Experiment with different filter types and settings