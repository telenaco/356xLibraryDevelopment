/**
 * @file mcp356x_multiple_channel_operations.cpp
 * @brief Demonstrates reading multiple channels from multiple MCP356x ADCs.
 *
 * This example configures up to three MCP356x ADCs and reads data from all differential channels
 * on each ADC. It shows how to initialize multiple ADCs with different pin configurations,
 * read values from all channels, and measure performance metrics such as timing. The example 
 * outputs detailed readings and timing information for each scan cycle.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to all MCP356x SDI pins
 * - SDO_PIN (12): Connect to all MCP356x SDO pins
 * - SCK_PIN (13): Connect to all MCP356x SCK pins
 * - ADC1_CS_PIN (2): Connect to first MCP356x CS pin
 * - ADC1_IRQ_PIN (3): Connect to first MCP356x IRQ pin
 * - ADC2_CS_PIN (4): Connect to second MCP356x CS pin
 * - ADC2_IRQ_PIN (5): Connect to second MCP356x IRQ pin
 * - ADC3_CS_PIN (7): Connect to third MCP356x CS pin
 * - ADC3_IRQ_PIN (6): Connect to third MCP356x IRQ pin
 */

#include "Arduino.h"
#include "MCP356x.h"

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pin definitions for multiple ADCs
#define ADC1_CS_PIN  2
#define ADC1_IRQ_PIN 3
#define ADC2_CS_PIN  4
#define ADC2_IRQ_PIN 5
#define ADC3_CS_PIN  7
#define ADC3_IRQ_PIN 6

/**
 * @brief Configuration structures for each MCP356x ADC
 */
// Configuration for the first ADC
MCP356xConfig config1 = {
    .irq_pin = ADC1_IRQ_PIN,                       // IRQ pin for first ADC
    .cs_pin = ADC1_CS_PIN,                         // CS pin for first ADC
    .mclk_pin = 0,                                 // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                  // Device address (GND in this case)
    .spiInterface = &SPI,                          // SPI interface to use
    .numChannels = 4,                              // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,       // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                   // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE    // Continuous conversion mode
};

// Configuration for the second ADC
MCP356xConfig config2 = {
    .irq_pin = ADC2_IRQ_PIN,                       // IRQ pin for second ADC
    .cs_pin = ADC2_CS_PIN,                         // CS pin for second ADC
    .mclk_pin = 0,                                 // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                  // Device address (GND in this case)
    .spiInterface = &SPI,                          // SPI interface to use
    .numChannels = 4,                              // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,       // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                   // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE    // Continuous conversion mode
};

// Configuration for the third ADC
MCP356xConfig config3 = {
    .irq_pin = ADC3_IRQ_PIN,                       // IRQ pin for third ADC
    .cs_pin = ADC3_CS_PIN,                         // CS pin for third ADC
    .mclk_pin = 0,                                 // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                  // Device address (GND in this case)
    .spiInterface = &SPI,                          // SPI interface to use
    .numChannels = 4,                              // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,       // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                   // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE    // Continuous conversion mode
};

// Pointers for dynamically created ADC objects
MCP356x* adc1 = nullptr;
MCP356x* adc2 = nullptr;
MCP356x* adc3 = nullptr;

/**
 * @brief Reads and prints all channels from one ADC
 * 
 * @param adc Pointer to the MCP356x ADC to read from
 * @param adcName Name identifier for this ADC in output
 */
void readAndPrintADC(MCP356x* adc, const char* adcName) {
    if (adc->updatedReadings()) {
        // Array of differential channels to read
        MCP356xChannel diffChannels[] = { 
            MCP356xChannel::DIFF_A, 
            MCP356xChannel::DIFF_B, 
            MCP356xChannel::DIFF_C, 
            MCP356xChannel::DIFF_D 
        };
        
        // Buffer for formatted output
        char text[350];

        // Iterate through the differential channels
        for (int i = 0; i < 4; ++i) {
            // Get the raw ADC value for this channel
            int32_t adcValue = adc->value(diffChannels[i]);
            
            // Format and print the channel value
            snprintf(text, sizeof(text), "%s Channel %d (DIFF_%c): %ld", 
                     adcName, i, 'A' + i, adcValue);
            Serial.println(text);
        }
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and multiple MCP356x ADCs.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize each MCP356x ADC
    adc1 = new MCP356x(config1);
    adc2 = new MCP356x(config2);
    adc3 = new MCP356x(config3);
}

/**
 * @brief Main program loop
 * 
 * Periodically reads data from all ADCs, tracking timing information,
 * and outputs combined readings and performance metrics.
 */
void loop() {
    // Variables for timing and performance tracking
    static unsigned long lastReadTime = 0;    // Stores the last time readings were taken
    unsigned long currentTime = micros();

    // Check if all ADCs have new data available and are ready
    if ((2 == adc1->read()) && (2 == adc2->read()) && (2 == adc3->read())) {
        // Capture start time of reading cycle
        unsigned long startTime = micros();

        // Prepare buffer and read values from all ADCs
        char buffer[512];    // Buffer for formatted output

        // Format all readings into a single output string
        snprintf(buffer, sizeof(buffer),
                "ADC1: [%ld, %ld, %ld, %ld], ADC2: [%ld, %ld, %ld, %ld], ADC3: [%ld, %ld, %ld, %ld]",
                adc1->value(MCP356xChannel::DIFF_A), adc1->value(MCP356xChannel::DIFF_B),
                adc1->value(MCP356xChannel::DIFF_C), adc1->value(MCP356xChannel::DIFF_D),
                adc2->value(MCP356xChannel::DIFF_A), adc2->value(MCP356xChannel::DIFF_B),
                adc2->value(MCP356xChannel::DIFF_C), adc2->value(MCP356xChannel::DIFF_D),
                adc3->value(MCP356xChannel::DIFF_A), adc3->value(MCP356xChannel::DIFF_B),
                adc3->value(MCP356xChannel::DIFF_C), adc3->value(MCP356xChannel::DIFF_D));
        
        // Output the combined readings
        Serial.println(buffer);
        
        // Calculate and print timing information
        unsigned long endTime = micros();
        Serial.print("Time for readings: ");
        Serial.print(endTime - startTime);
        Serial.println(" microseconds");
        
        // Print the time since the last reading cycle
        Serial.print("Time since last readings: ");
        Serial.print(startTime - lastReadTime);
        Serial.println(" microseconds");
        
        // Update lastReadTime for the next cycle
        lastReadTime = startTime;
    }
}