/**
 * @file mcp356x_multiple_filter_comparison.cpp
 * @brief Demonstrates the performance of multiple filters using the MCP356x ADC.
 *
 * This program reads data from the MCP356x ADC and applies various filters to the raw readings.
 * The filters used include Butterworth, Simple Moving Average, Median, Notch, and a custom running average.
 * The filtered values, along with the raw reading and timestamp, are printed to the serial monitor
 * for comparison of filter performance and characteristics.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - ADC_IRQ_PIN (6): Connect to MCP356x IRQ (Interrupt Request)
 * - ADC_CS_PIN (7): Connect to MCP356x CS (Chip Select)
 */

#include "MCP356x.h"
#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/MedianFilter.hpp>
#include <Filters/Butterworth.hpp>
#include <Filters/Notch.hpp>
#include <Filters/SMA.hpp>
#include <AH/STL/cmath>

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN  7

// Channel to read
const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,                            // Interrupt Request pin
    .cs_pin = ADC_CS_PIN,                              // Chip select pin
    .mclk_pin = 0,                                     // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                      // Device address (GND in this case)
    .spiInterface = &SPI,                              // SPI interface to use
    .numChannels = 1,                                  // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,           // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                       // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE        // Continuous conversion mode
};

// Pointer for dynamically created ADC object
MCP356x* scale = nullptr;

// Buffer for custom running average filter
const int BUFFER_SIZE = 50;
int buffer[BUFFER_SIZE];
int head = 0;
bool bufferFull = false;
int runningSum = 0;

/**
 * @brief Computes the running average of the input values.
 *
 * This function maintains a circular buffer of size BUFFER_SIZE and calculates
 * the running average of the values in the buffer. It updates the buffer and
 * the running sum with each new value and returns the current average.
 *
 * @param value The new value to be added to the running average calculation.
 * @return The current running average of the values in the buffer.
 */
int computeRunningAverage(int value) {
    if (bufferFull) {
        runningSum -= buffer[head];
    }
    
    runningSum += value;
    buffer[head] = value;
    head = (head + 1) % BUFFER_SIZE;
    
    if (head == 0) {
        bufferFull = true;
    }
    
    if (bufferFull) {
        return runningSum / BUFFER_SIZE;
    } else {
        return runningSum / head;
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize the ADC
    scale = new MCP356x(config);
    
    // Print CSV header for data output
    Serial.println("Timestamp,RawValue,Butterworth,SMA,Median,Notch,CustomAverage");
}

// Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11111;  // Hz (based on ADC sampling rate)
constexpr double CUT_OFF_FREQUENCY = 10;      // Hz (filter cutoff frequency)
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
constexpr double NOTCH_FREQUENCY = 50;        // Hz (notch filter frequency)
constexpr double NORMALIZED_NOTCH = 2 * NOTCH_FREQUENCY / SAMPLING_FREQUENCY;

// Initialize filter instances
Timer<micros> sampleTimer = std::round(1e6 / SAMPLING_FREQUENCY);
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);
SMA<50, int32_t, int32_t> simpleMovingAvg = {0};
MedianFilter<50, int32_t> medianFilter = {0};
auto primaryNotchFilter = simpleNotchFIR(NOTCH_FREQUENCY);
auto secondaryNotchFilter = simpleNotchFIR(0.06360676);

/**
 * @brief Main program loop
 * 
 * Continuously reads data from the ADC, applies various filters,
 * and outputs the raw and filtered values for comparison.
 */
void loop() {
    // Check if the ADC has new data available
    if (scale->updatedReadings()) {
        // Get raw ADC reading
        int rawReading = scale->value(LOADCELL_CHANNEL);
        
        // Apply different filters to the raw reading
        float butterworthFilteredValue = butterworthFilter(rawReading);
        int averagedValue = simpleMovingAvg(rawReading);
        int medianFilteredValue = medianFilter(rawReading);
        float notchedValue = secondaryNotchFilter(primaryNotchFilter(rawReading));
        int customRunningAverage = computeRunningAverage(rawReading);

        // Format and output all filter results
        StringBuilder output;
        output.concatf("%u,%d,%d,%d,%d,%d,%d",
                       micros(),
                       rawReading,
                       static_cast<int>(butterworthFilteredValue),
                       averagedValue,
                       medianFilteredValue,
                       static_cast<int>(notchedValue),
                       customRunningAverage);

        Serial.println((char*)output.string());
    }
    
    // Small delay to prevent overwhelming the serial output
    delayMicroseconds(90);
}