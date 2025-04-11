/**
 * @file mcp356x_sampling_rate_measurement.cpp
 * @brief Demonstrates and measures the sampling rate of the MCP356x ADC.
 *
 * This example configures the MCP356x ADC for continuous sampling on a differential channel
 * and measures the actual sampling rate achieved. It counts conversions over a 5-second interval,
 * calculates the samples per second and average time per sample, and outputs detailed timing
 * information. This is useful for performance analysis and verification of ADC settings.
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
#include <SPI.h>

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 7
#define ADC_IRQ_PIN 6
#define MCLK_PIN 0

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,                     // Interrupt Request pin
    .cs_pin = ADC_CS_PIN,                       // Chip Select pin
    .mclk_pin = MCLK_PIN,                       // Master Clock pin (0 for internal clock)
    .addr = 0x01,                               // Device address (GND in this case)
    .spiInterface = &SPI,                       // SPI interface to use
    .numChannels = 1,                           // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE // Continuous conversion mode
};

// Pointer for dynamically created ADC object
MCP356x *adc = nullptr;

// Variables for timing and sample counting
unsigned long startTime;
unsigned int sampleCount = 0;

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 * Configures a single differential channel for sampling.
 */
void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    Serial.print("\n\n");

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize the ADC
    adc = new MCP356x(config);

    // Configure the ADC for a single differential channel
    adc->setScanChannels(1, MCP356xChannel::DIFF_A);
    // Uncomment below line to scan multiple channels instead
    // adc->setScanChannels(4, MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D);

    // Initialize the start time for sampling rate calculation
    startTime = micros();
}

/**
 * @brief Main program loop
 *
 * Counts ADC conversions and calculates sampling statistics every 5 seconds.
 * Outputs the samples per second, average time per sample, and detailed timing information.
 */
void loop()
{
    // Check if the ADC has new data available
    if (adc->updatedReadings())
    {
        sampleCount++;
    }

    // Every 5 seconds, calculate and display sampling statistics
    if (micros() - startTime >= 5000000)
    { // 5 seconds in microseconds
        // Calculate samples per second and average time per sample
        float samplesPerSecond = sampleCount / 5.0;
        float averageTimePerSample = 5000000.0 / sampleCount;

        // Prepare the output string with sampling statistics
        StringBuilder output;
        output.concatf("Updates per second: %.2f samples/s\n", samplesPerSecond);
        output.concatf("Average time per update: %.2f us/sample\n", averageTimePerSample);

        // Add detailed timing information from the ADC
        adc->printTimings(&output);

        // Output the sampling statistics to serial
        Serial.print((char *)output.string());

        // Reset for the next interval
        startTime = micros();
        sampleCount = 0;
    }
}