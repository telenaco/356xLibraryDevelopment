/**
 * @file ADC_Readings_With_Filter.ino
 * @brief ADC Reading with Butterworth Filter
 * 
 * This script reads data from an MCP356x ADC and applies a Butterworth filter
 * to the readings. The main goal is to capture data from the ADC, filter it, 
 * and then send the filtered and raw data to the serial port.
 * 
 * The script captures and prints the following details:
 *   - Current microsecond timestamp
 *   - Raw ADC reading
 *   - Butterworth-filtered value
 *   - Loop counter (for tracking iterations)
 *   - Elapsed time since the last successful ADC reading
 * 
 * An explicit delay is added to ensure consistent serial print messages 
 * and compensate for ADC read time.
 * 
 * Note: Ensure that all required libraries, including Filters and MCP356x, are installed.
 * 
 * @author [Your Name]
 * @date [Date of Creation]
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

// MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define ADC_CS_PIN 2  // Chip select for ADC
#define ADC_IRQ_PIN 3 // Interrupt for ADC
#define MCLK_PIN 0    //

const MCP356xChannel LOADCELL_CHANNEL =
    MCP356xChannel::DIFF_A; // Using DIFF_A channel

MCP356x scale(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);

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
  scale.tare(LOADCELL_CHANNEL);
  delay(500);
}

// Filter Configuration
constexpr double SAMPLING_FREQUENCY = 11111;
constexpr double CUT_OFF_FREQUENCY = 2;
constexpr double NORMALIZED_CUT_OFF =
    2 * CUT_OFF_FREQUENCY / SAMPLING_FREQUENCY;

auto butterworthFilter = butter<1>(NORMALIZED_CUT_OFF);

void loop() {
  static uint32_t lastSuccessfulReadTime = 0; // Time of the last successful read
  static uint32_t loopCounter = 0;  // Counter to keep track of loop iterations

  loopCounter++;  // Increment the loop counter at the start of each loop iteration
  
  if (scale.isr_fired && scale.read() == 2) {
    int rawReading = scale.getDigitalValue(LOADCELL_CHANNEL);
    float butterworthFilteredValue = butterworthFilter(rawReading);
    
    uint32_t currentMicros = micros(); // Current time
    uint32_t elapsedMicros = currentMicros - lastSuccessfulReadTime; // Calculate elapsed time since the last successful read

    char output[70]; // Adjusted size to accommodate the loop counter and elapsed microseconds

    snprintf(output, sizeof(output), "%lu,%d,%d,%lu,%lu", 
            currentMicros, 
            rawReading,
            static_cast<int>(butterworthFilteredValue), 
            loopCounter, 
            elapsedMicros);

    Serial.println(output);
    Serial.flush();

    lastSuccessfulReadTime = currentMicros; // Update the time of the last successful read
  }
  delayMicroseconds(90); // it takes about 70 microseconds for a read to be ready, this delay allows for consistent serial print messages
}
