/**
 * @file 12_mcpVshx711_force.cpp
 * @brief Compares raw and filtered readings from MCP356x and HX711 sensors.
 *
 * This program interfaces with MCP356x and HX711 sensors to measure load cell data.
 * It captures raw readings and processes them through a Butterworth filter. The primary
 * purpose is to compare the readings from both sensors side by side.
 *
 * The output consists of:
 * - Timestamp (microseconds since the program started)
 * - Raw reading from MCP356x
 * - Filtered reading from MCP356x
 * - Raw reading from HX711
 * - Filtered reading from HX711
 *
 * This allows for a comprehensive comparison between the two sensors, both in terms
 * of raw values and the effect of the filtering process.
 */

#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"
#include "MCP356xScale.h"

// Pins Configuration
const uint8_t SDI_PIN         = 11;
const uint8_t SDO_PIN         = 12;
const uint8_t SCK_PIN         = 13;
const uint8_t MCP_ADC_CS_PIN  = 7;
const uint8_t MCP_ADC_IRQ_PIN = 6;
const uint8_t HX711_DOUT_PIN  = 23;
const uint8_t HX711_SCK_PIN   = 22;

// MCP Filter Configuration
constexpr double MCP_SAMPLING_FREQUENCY = 11000;
constexpr double MCP_CUT_OFF_FREQUENCY  = 48;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// HX711 Filter Configuration
constexpr double HX711_SAMPLING_FREQUENCY = 80;
constexpr double HX711_CUT_OFF_FREQUENCY  = 10;
constexpr double HX711_NORMALIZED_CUT_OFF = 2 * HX711_CUT_OFF_FREQUENCY / HX711_SAMPLING_FREQUENCY;

// ====== Variables ======
MCP356xScale* mcpScale = nullptr;
HX711 hx711Scale;

auto butterworthMcpFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
auto butterworthHxFilter  = butter<1>(HX711_NORMALIZED_CUT_OFF);

volatile boolean hx711DataReady = false;

int32_t mcpReading           = 0;
int32_t hx711Reading         = 0;
float filteredMcpReading   = 0;
float filteredHx711Reading = 0;

// ====== Function Definitions ======
void hx711DataReadyISR() {
  if (hx711Scale.is_ready()) {
    hx711DataReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  mcpScale = new MCP356xScale(1, SCK_PIN, SDO_PIN, SDI_PIN, MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN);

  // MCP Calibration Configuration
  mcpScale->setScaleFactor(0, 0.0006093949987217794f);
  // mcpScale->setLinearCalibration(0, 0.0006025509378972841f, -538.4288848531679f);
  // mcpScale->setPolynomialCalibration(0, -9.500760302035329e-14f, 0.0006027208452100216f, -538.3644732675577f);
  mcpScale->tare(0);

  hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);

  // HX711 Calibration Configuration
  hx711Scale.set_scale(0.0037841729238619852f);
  hx711Scale.tare(10);
}

void loop() {
  bool printOutput = false;

  if (mcpScale->updatedAdcReadings()) {
    mcpReading = mcpScale->getReading(0);
    filteredMcpReading = butterworthMcpFilter(mcpReading);
    printOutput = true;
  }

  if (hx711DataReady) {
    hx711Reading = hx711Scale.get_units();
    filteredHx711Reading = butterworthHxFilter(hx711Reading);
    hx711DataReady = false;
    printOutput = true;
  }

  if (printOutput) {
    unsigned long elapsedTime = micros();
    StringBuilder output;
    output.concatf("%lu, %ld, %.2f, %ld, %.2f",
                   elapsedTime,
                   mcpReading,
                   filteredMcpReading,
                   hx711Reading,
                   filteredHx711Reading);

    Serial.println((char*)output.string());
  }
}