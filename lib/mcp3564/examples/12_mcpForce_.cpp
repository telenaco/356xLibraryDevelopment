/**
 * @file MCP356x_LoadCell.cpp
 * @brief Interface and control routines for MCP356x ADC with load cell data processing.
 *
 * This file contains functions to interface with the MCP356x ADC, read data from a load cell,
 * apply necessary calibrations, and filter the readings. The filtered and calibrated readings
 * are then outputted over the serial interface.
 */

#include "MCP356xScale.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN  7
#define IRQ_PIN 6

// Filter Configuration for MCP356x
constexpr double MCP_SAMPLING_FREQUENCY = 11111;
constexpr double MCP_CUT_OFF_FREQUENCY = 11;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// Global Variables
MCP356xScale* mcpScale              = nullptr;
auto butterworthFilter              = butter<1>(MCP_NORMALIZED_CUT_OFF);
int32_t mcpRawReading               = 0;
float mcpFilteredReading            = 0;
uint32_t lastSuccessfulMCPReadTime  = 0;
uint32_t elapsedMCPMicros           = 0;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Initialize the MCP356xScale with 1 ADC and 1 scale
  mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN, CS_PIN);

  // MCP Calibration Configuration
  mcpScale->setScaleFactor(0, 0.0006017f);
  // mcpScale->setLinearCalibration(0, 0.0006028f, -538.524f);
  // mcpScale->setPolynomialCalibration(0, 0.0f, 0.0006026f, -538.554f);
  mcpScale->tare(0);
}

void loop() {
  static uint32_t lastPrintTime = 0;

  // Update ADC readings
  if (mcpScale->updatedAdcReadings()) {
    elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
    mcpRawReading = mcpScale->getReading(0);
    mcpFilteredReading = butterworthFilter(mcpRawReading);
    lastSuccessfulMCPReadTime = micros();
  }

  // Get the current time
  uint32_t currentTime = millis();

  // Print the readings every 500ms
  if (currentTime - lastPrintTime >= 500) {
    lastPrintTime = currentTime;

    // Prepare the output string
    StringBuilder output;
    output.concatf("Raw Reading: %ld, Filtered Reading: %.2f grams, Time Elapsed: %lu microseconds",
      mcpRawReading, mcpFilteredReading, elapsedMCPMicros);

    // Output the readings
    Serial.println((char*)output.string());
  }
}