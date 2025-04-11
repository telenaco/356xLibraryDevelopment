/**
 * @file mcp356x_basic_differential_reading.cpp
 * @brief Basic example demonstrating differential channel reading with the MCP356x ADC.
 *
 * This example demonstrates the fundamental operation of the MCP356x ADC for reading a 
 * differential channel. It configures the ADC, initializes SPI communication, and 
 * periodically reads both raw ADC values and voltage measurements from the DIFF_A channel.
 * The readings are printed to the serial console every 5 seconds, along with timing information.
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

#include "Arduino.h"
#include "MCP356x.h"

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN 7

// Global variables
unsigned long startTime;

/**
 * @brief MCP356x ADC configuration structure
 * 
 * This structure defines the hardware connections and operational parameters
 * for the MCP356x ADC.
 */
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,                           // Interrupt Request pin
    .cs_pin = ADC_CS_PIN,                             // Chip Select pin
    .mclk_pin = 0,                                    // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                     // Device address (GND in this case)
    .spiInterface = &SPI,                             // SPI interface to use
    .numChannels = 1,                                 // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,          // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                      // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE       // Continuous conversion mode
};

// Pointer for dynamically created ADC object
MCP356x* adc = nullptr;

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);  // Wait for serial port to connect
    }

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Alternative pin configurations (commented out)
    // config.irq_pin = 3; config.cs_pin = 2; // For a second ADC
    // config.irq_pin = 5; config.cs_pin = 4; // For a third ADC
    
    // Create and initialize the ADC
    adc = new MCP356x(config);
    
    // Set initial time reference
    startTime = millis();
}

/**
 * @brief Main program loop
 * 
 * Continuously checks for updated ADC readings. Every 5 seconds,
 * reads voltage and raw ADC values and prints them to the serial console.
 */
void loop() {
    // Check for updated ADC readings (polling approach)
    // Uncomment the below block to implement polling
    /*
    if (adc->updatedReadings()) {
        // Process new readings here if needed
    }
    */

    // Every 5 seconds, read and print ADC values
    if (millis() - startTime >= 5000) {
        // Get voltage value from the differential channel A
        float voltage = adc->valueAsVoltage(MCP356xChannel::DIFF_A);

        // Get raw ADC value
        int32_t adcValue = adc->value(MCP356xChannel::DIFF_A);

        // Format and print the ADC value, voltage, and timing information
        StringBuilder output;
        output.concatf("ADC Value: %ld, Voltage: %.2fV\n", adcValue, voltage);
        
        // Print detailed timing information
        adc->printTimings(&output);

        // Output the formatted string
        if (output.length() > 0) {
            Serial.print((char*)output.string());
        }
        
        // Reset timer for the next interval
        startTime = millis();
    }
}