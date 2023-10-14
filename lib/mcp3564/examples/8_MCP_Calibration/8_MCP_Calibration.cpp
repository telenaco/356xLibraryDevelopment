/**
 * Filename: loadcell_readings.cpp
 * 
 * Description:
 *     This code captures readings from an MCP356x-based load cell. The readings
 *     are filtered using a Butterworth filter to minimize noise and enhance precision.
 *     The raw and filtered readings are then sent over the serial port for further processing.
 * 
 * Requirements:
 *     - MCP356x library
 *     - Filters library
 * 
 * Author: [Your Name]
 * Date: [Date of Creation]
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// MCP356x Pin Configuration
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2  // Chip select for ADC
#define ADC_IRQ_PIN 3 // Interrupt for ADC
#define MCLK_PIN 0

const MCP356xChannel LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

MCP356x scale(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);

// Function to set up the ADC
void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, LOADCELL_CHANNEL);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void setup() {
  Serial.begin(2000000);
  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();
  setupADC(scale);
}

// Butterworth Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11000;  // since are taking a sample every ~90 uSeconds freq is 1/T
constexpr double CUT_OFF_FREQUENCY = 48;
constexpr double NORMALIZED_CUT_OFF = 2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;
auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

void loop() {
  if (scale.isr_fired && scale.read() == 2) {
    int32_t reading = scale.value(LOADCELL_CHANNEL);
    
    // Apply the Butterworth filter
    int32_t filteredValue = butterworthFilter(reading);
    
    // Print the raw and filtered readings to the serial port
    Serial.print(reading);
    Serial.print(", ");
    Serial.println(filteredValue);
  }
  
  delayMicroseconds(90);  // Delay for consistent readings
}
