/**
 * @file calibration_mcp356x_basic.cpp
 * @brief Captures readings from MCP356x ADC for load cell calibration.
 *
 * This program captures raw and filtered readings from an MCP356x ADC connected to a load cell.
 * The data is sent to the serial port for calibration analysis. Butterworth filtering is applied
 * to minimize noise and enhance precision. This example is designed to be used with external
 * calibration software to generate calibration constants for the load cell.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - ADC_CS_PIN (2): Connect to MCP356x CS (Chip Select)
 * - ADC_IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 * - MCLK_PIN (0): Connect to MCP356x MCLK (Master Clock) - optional
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2   // Chip select for ADC
#define ADC_IRQ_PIN 3  // Interrupt for ADC
#define MCLK_PIN 0     // Master Clock pin (0 for internal clock)

// ADC channel configuration
const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig config = {
    .irq_pin      = ADC_IRQ_PIN,                        // Interrupt Request pin
    .cs_pin       = ADC_CS_PIN,                         // Chip Select pin
    .mclk_pin     = MCLK_PIN,                           // Master Clock pin
    .addr         = 0x01,                               // Device address (GND in this case)
    .spiInterface = &SPI,                               // SPI interface to use
    .numChannels  = 1,                                  // Number of channels to scan
    .osr          = MCP356xOversamplingRatio::OSR_32,   // Oversampling ratio
    .gain         = MCP356xGain::GAIN_1,                // Gain setting (1x)
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE // Continuous conversion mode
};

// Pointer for dynamically created ADC object
MCP356x* scale = nullptr;

/**
 * @brief Set up the ADC with appropriate configuration
 * 
 * Configures the ADC to use internal clock and scan the loadcell channel
 */
void setupADC() {
    scale->setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    scale->setScanChannels(1, LOADCELL_CHANNEL);
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 */
void setup() {
    // Initialize serial communication with high baud rate for fast data transfer
    Serial.begin(2000000);
    
    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize the ADC
    scale = new MCP356x(config);
    setupADC();
}

// Butterworth Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11000;  // Hz (based on ADC sampling rate)
constexpr double CUT_OFF_FREQUENCY = 48;      // Hz (filter cutoff frequency)
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

/**
 * @brief Main program loop
 * 
 * Continuously reads from the ADC, applies filtering, and sends data to serial port.
 * The output format is "raw_reading, filtered_reading" for each sample.
 */
void loop() {
    // Check if the ADC has new data available
    if (scale->updatedReadings()) {
        // Get raw ADC reading
        int32_t reading = scale->value(LOADCELL_CHANNEL);
        
        // Apply the Butterworth filter to reduce noise
        int32_t filteredValue = butterworthFilter(reading);

        // Format and output the readings
        StringBuilder output;
        output.concatf("%d, %d", reading, filteredValue);
        Serial.println((char*)output.string());
        Serial.flush();  // Ensure data is sent immediately
    }
}
