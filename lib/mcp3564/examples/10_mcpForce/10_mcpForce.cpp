/**
 * @file MCP356x.cpp
 * @brief Interface and control routines for MCP356x ADC with load cell data processing.
 * 
 * This file contains functions to interface with the MCP356x ADC, read data from a load cell,
 * apply necessary calibrations, and filter the readings. The filtered and calibrated readings
 * are then outputted over the serial interface.
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// ============== Constants ==============
// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define MCP_ADC_CS_PIN 2  // Chip select for MCP356x ADC
#define MCP_ADC_IRQ_PIN 3 // Interrupt for MCP356x ADC
#define MCLK_PIN 0

// MCP356x Channel Definition
const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

// Filter Configuration for MCP356x
constexpr double MCP_SAMPLING_FREQUENCY = 11111;
constexpr double MCP_CUT_OFF_FREQUENCY = 11;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// ============== Global Variables ==============
MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);
auto butterworthFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
int mcpRawReading = 0;

// ============== Function Definitions ==============
void setupADC(MCP356x &adc) {
    adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    adc.init(&SPI);
    adc.setScanChannels(1, MCP_LOADCELL_CHANNEL);
    adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
    adc.setGain(MCP356xGain::GAIN_1);
    adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);

    // Setup SPI Configuration
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // ADC Setup
    setupADC(mcpScale);
    mcpScale.tare(MCP_LOADCELL_CHANNEL);

    // MCP Calibration Configuration
    mcpScale.setScaleFactor(MCP_LOADCELL_CHANNEL, 0.0006016569289837716f);
    mcpScale.setLinearCalibration(MCP_LOADCELL_CHANNEL, 0.0006027751375968074f, -538.5241383004725f);
    mcpScale.setPolynomialCalibration(MCP_LOADCELL_CHANNEL, 8.095189745489105e-14f, 0.0006025713062654753f, -538.5539280900973f);
    mcpScale.setConversionMode(MCP_LOADCELL_CHANNEL, MCP356xConversionMode::POLYNOMIAL);
}

void loop() {
    // Check ADC readings
    if (mcpScale.isr_fired && mcpScale.read() == 2) {
        mcpRawReading = mcpScale.getForce(MCP_LOADCELL_CHANNEL);
        float filteredReading = butterworthFilter(mcpRawReading); // Filter the ADC readings

        // Prepare the output string
        char output[50];
        snprintf(output, sizeof(output), "%d, %d", mcpRawReading, static_cast<int>(filteredReading));

        // Output the readings
        Serial.println(output);
        Serial.flush();
    }

    // Delay for ADC to settle
    delayMicroseconds(90);
}
