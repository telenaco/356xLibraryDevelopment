/**
 * @file mcp356x_multiple_adc_sampling.cpp
 * @brief Demonstrates continuous sampling using multiple MCP356x ADCs.
 *
 * This example configures three MCP356x ADCs for continuous sampling on differential channels.
 * It counts conversions from each ADC over a 5-second interval and calculates sampling rates
 * for each ADC, as well as the combined sampling rate. The example demonstrates how to manage
 * multiple ADCs simultaneously and how to read multiple channels from each ADC.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to all MCP356x SDI pins
 * - SDO_PIN (12): Connect to all MCP356x SDO pins
 * - SCK_PIN (13): Connect to all MCP356x SCK pins
 * - ADC0_IRQ_PIN (6): Connect to MCP356x ADC0 IRQ pin
 * - ADC0_CS_PIN (7): Connect to MCP356x ADC0 CS pin
 * - ADC1_IRQ_PIN (5): Connect to MCP356x ADC1 IRQ pin
 * - ADC1_CS_PIN (4): Connect to MCP356x ADC1 CS pin
 * - ADC2_IRQ_PIN (3): Connect to MCP356x ADC2 IRQ pin
 * - ADC2_CS_PIN (2): Connect to MCP356x ADC2 CS pin
 */

#include "MCP356x.h"

// Pin definitions for SPI communication
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pin definitions for the three ADCs
#define ADC0_CS_PIN 7
#define ADC1_CS_PIN 4
#define ADC2_CS_PIN 2

#define ADC0_IRQ_PIN 6
#define ADC1_IRQ_PIN 5
#define ADC2_IRQ_PIN 3

/**
 * @brief Base MCP356x ADC configuration structure
 * 
 * This structure defines the common settings for all ADCs.
 * Specific pins are updated for each ADC instance.
 */
MCP356xConfig config = {
    .irq_pin = ADC0_IRQ_PIN,                       // Default IRQ pin (updated for each ADC)
    .cs_pin = ADC0_CS_PIN,                         // Default CS pin (updated for each ADC)
    .mclk_pin = 0,                                 // Master Clock pin (0 for internal clock)
    .addr = 0x01,                                  // Device address (GND in this case)
    .spiInterface = &SPI,                          // SPI interface to use
    .numChannels = 4,                              // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,       // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                   // Gain setting (1x)
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE    // Continuous conversion mode
};

// Pointers for dynamically created ADC objects
MCP356x* adc0 = nullptr;
MCP356x* adc1 = nullptr;
MCP356x* adc2 = nullptr;

// Variables for timing and sample counting
unsigned long startTime;
unsigned int sampleCount0 = 0;
unsigned int sampleCount1 = 0;
unsigned int sampleCount2 = 0;

/**
 * @brief Reads data from an ADC and increments its sample counter
 * 
 * @param adc Pointer to the MCP356x ADC to read from
 * @param sampleCounter Reference to the sample counter for this ADC
 */
void readADC(MCP356x& adc, unsigned int& sampleCounter) {
    if (adc.updatedReadings()) {
        // Read voltage values from all four differential channels
        float voltageA = adc.valueAsVoltage(MCP356xChannel::DIFF_A);
        float voltageB = adc.valueAsVoltage(MCP356xChannel::DIFF_B);
        float voltageC = adc.valueAsVoltage(MCP356xChannel::DIFF_C);
        float voltageD = adc.valueAsVoltage(MCP356xChannel::DIFF_D);
        
        // Increment the sample counter for this ADC
        sampleCounter++;
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and three MCP356x ADCs.
 * Configures each ADC for sampling four differential channels.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.print("\n\n");

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Create and initialize the first ADC
    adc0 = new MCP356x(config);
    
    // Update configuration and create the second ADC
    config.cs_pin = ADC1_CS_PIN;
    config.irq_pin = ADC1_IRQ_PIN;
    adc1 = new MCP356x(config);
    
    // Update configuration and create the third ADC
    config.cs_pin = ADC2_CS_PIN;
    config.irq_pin = ADC2_IRQ_PIN;
    adc2 = new MCP356x(config);

    // Configure each ADC to scan all four differential channels
    adc0->setSpecificChannels(4);
    adc1->setSpecificChannels(4);
    adc2->setSpecificChannels(4);

    // Initialize the start time for sampling rate calculation
    startTime = micros();
}

/**
 * @brief Main program loop
 * 
 * Reads data from all three ADCs, counts samples, and calculates sampling rates
 * every 5 seconds. Outputs individual and total kilosamples per second (ksps).
 */
void loop() {
    // Check each ADC for new data and update sample counts
    if (adc0->updatedReadings()) {
        readADC(*adc0, sampleCount0);
    }
    if (adc1->updatedReadings()) {
        readADC(*adc1, sampleCount1);
    }
    if (adc2->updatedReadings()) {
        readADC(*adc2, sampleCount2);
    }

    // Every 5 seconds, calculate and display sampling statistics
    if (millis() - startTime >= 5000) {
        // Calculate samples per second for each ADC in kilosamples per second (ksps)
        float ksps0 = sampleCount0 / 5.0 / 1000.0;
        float ksps1 = sampleCount1 / 5.0 / 1000.0;
        float ksps2 = sampleCount2 / 5.0 / 1000.0;
        float totalKsps = ksps0 + ksps1 + ksps2;

        // Prepare and output the sampling statistics
        StringBuilder output;
        output.concatf("ADC0 ksps: %.2f\n", ksps0);
        output.concatf("ADC1 ksps: %.2f\n", ksps1);
        output.concatf("ADC2 ksps: %.2f\n", ksps2);
        output.concatf("Total ksps: %.2f\n", totalKsps);
        Serial.print((char*)output.string());

        // Reset for the next interval
        startTime = millis();
        sampleCount0 = 0;
        sampleCount1 = 0;
        sampleCount2 = 0;
    }
}