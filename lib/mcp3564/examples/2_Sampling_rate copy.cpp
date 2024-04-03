/**
 * @file 2_Sampling_rate.cpp
 * @brief Demonstrates continuous sampling and calculates the sampling rate using the MCP356x ADC.
 *
 * This program configures the MCP356x ADC for continuous sampling on a single differential channel (DIFF_A).
 * It triggers an interrupt when each conversion is complete. The main loop counts the number of conversions
 * over a 5-second interval and calculates the sample rate.
 *
 * Hardware Connections: 
 * - SDI_PIN           : Connect to MCP356x SDI (Serial Data Input).
 * - SDO_PIN           : Connect to MCP356x SDO (Serial Data Output).
 * - SCK_PIN           : Connect to MCP356x SCK (Serial Clock Input).
 * - ADC_IRQ_PIN       : Connect to MCP356x IRQ (Interrupt Request).
 * - ADC_CS_PIN        : Connect to MCP356x CS (Chip Select).
 */

#include "MCP356x.h"
#include <SPI.h>

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_IRQ_PIN 6
#define ADC_CS_PIN  7


  // Define the configuration for the MCP356x
MCP356xConfig config = {
    .irq_pin      = ADC_IRQ_PIN,
    .cs_pin       = ADC_CS_PIN,
    .mclk_pin     = 0,
    .addr         = 0x01,
    .spiInterface = &SPI,
    .numChannels  = 1,
    .osr          = MCP356xOversamplingRatio::OSR_32,
    .gain         = MCP356xGain::GAIN_1,
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE
};

MCP356x* adc = nullptr;

unsigned long startTime;
unsigned int sampleCount = 0;

void setup() {
    Serial.begin(115200);
    Serial.print("\n\n");

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    adc = new MCP356x(config);
    startTime = micros();
}

void loop() {
    if (adc->updatedReadings()) {
        sampleCount++;
    }

    if (micros() - startTime >= 5000000) {
        float samplesPerSecond     = sampleCount / 5.0;
        float averageTimePerSample = 5000000.0 / sampleCount;

        StringBuilder output;
        output.concatf("updates per second: %.2f samples/s\n", samplesPerSecond);
        output.concatf("Average time per update: %.2f us/sample\n", averageTimePerSample);
        adc->printTimings(&output);

        Serial.print((char*)output.string());

        startTime   = micros();
        sampleCount = 0;
    }
}