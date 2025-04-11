/**
 * @file mcp356x_filtering_adc_readings.cpp
 * @brief ADC Reading with Butterworth Filter
 *
 * This program reads data from an MCP356x ADC and applies a Butterworth filter
 * to the readings. The main goal is to capture data from the ADC, filter it,
 * and then send the filtered and raw data to the serial port. The program tracks
 * timing between readings and provides detailed performance metrics.
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
#include <Filters/Butterworth.hpp>

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN 7
#define MCLK_PIN 0

// ADC channel configuration
const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,                     // Interrupt Request pin
    .cs_pin = ADC_CS_PIN,                       // Chip Select pin
    .mclk_pin = MCLK_PIN,                       // Master Clock pin
    .addr = 0x01,                               // Device address (GND in this case)
    .spiInterface = &SPI,                       // SPI interface to use
    .numChannels = 1,                           // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE // Continuous conversion mode
};

// Pointer for dynamically created ADC object
MCP356x *scale = nullptr;

/**
 * @brief Configure the ADC with appropriate settings
 *
 * Sets up the ADC to use internal clock and scan the loadcell channel.
 */
void setupADC()
{
    scale->setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    scale->setScanChannels(1, LOADCELL_CHANNEL);
}

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 */
void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize the ADC
    scale = new MCP356x(config);
    setupADC();

    // Print CSV header for data output
    Serial.println("Timestamp,RawReading,FilteredReading,LoopCounter,ElapsedMicros");
}

// Butterworth Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11111; // Hz (based on ADC sampling rate)
constexpr double CUT_OFF_FREQUENCY = 2;      // Hz (filter cutoff frequency)
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

/**
 * @brief Main program loop
 *
 * Continuously reads from the ADC, applies filtering, and outputs the raw and
 * filtered values along with timing information for performance analysis.
 */
void loop()
{
    static uint32_t lastSuccessfulReadTime = 0; // Time of the last successful read
    static uint32_t loopCounter = 0;            // Counter to keep track of loop iterations
    loopCounter++;

    // Check if the ADC has new data available
    if (scale->updatedReadings())
    {
        // Get raw ADC reading
        int32_t rawReading = scale->value(LOADCELL_CHANNEL);

        // Apply the Butterworth filter to the raw reading
        float butterworthFilteredValue = butterworthFilter(rawReading);

        // Get current timestamp and calculate elapsed time since last reading
        uint32_t currentMicros = micros();
        uint32_t elapsedMicros = currentMicros - lastSuccessfulReadTime;

        // Format and output the readings with timing information
        StringBuilder output;
        output.concatf("%lu,%d,%d,%lu,%lu",
                       currentMicros,
                       rawReading,
                       static_cast<int>(butterworthFilteredValue),
                       loopCounter,
                       elapsedMicros);

        Serial.println((char *)output.string());

        // Update the timestamp for the next reading
        lastSuccessfulReadTime = currentMicros;
    }
}