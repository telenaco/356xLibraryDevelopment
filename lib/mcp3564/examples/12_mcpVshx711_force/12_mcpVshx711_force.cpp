/**
 * \file LoadCellComparison.ino
 * \brief Compares raw and filtered readings from MCP356x and HX711 sensors.
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
#include "MCP356x.h"


// ====== Constants and Configuration ======
// Pins Configuration
const uint8_t SDI_PIN         = 11;
const uint8_t SDO_PIN         = 12;
const uint8_t SCK_PIN         = 13;
const uint8_t MCP_ADC_CS_PIN  = 2;
const uint8_t MCP_ADC_IRQ_PIN = 3;
const uint8_t MCLK_PIN        = 0;
const uint8_t HX711_DOUT_PIN  = 23;
const uint8_t HX711_SCK_PIN   = 22;

const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

// MCP Filter Configuration
constexpr double MCP_SAMPLING_FREQUENCY = 11000;
constexpr double MCP_CUT_OFF_FREQUENCY  = 48;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// HX711 Filter Configuration
constexpr double HX711_SAMPLING_FREQUENCY = 80;
constexpr double HX711_CUT_OFF_FREQUENCY  = 10;
constexpr double HX711_NORMALIZED_CUT_OFF = 2 * HX711_CUT_OFF_FREQUENCY / HX711_SAMPLING_FREQUENCY;

// ====== Variables ======
MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);
HX711 hx711Scale;

auto butterworthMcpFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
auto butterworthHxFilter  = butter<1>(HX711_NORMALIZED_CUT_OFF);

volatile boolean hx711DataReady = false;

int   mcpReading           = 0;
int   hx711Reading         = 0;
float filteredMcpReading   = 0;
float filteredHx711Reading = 0;

// ====== Function Definitions ======
void hx711DataReadyISR() {
  if (hx711Scale.is_ready()) {
    hx711DataReady = true;
  }
}

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, MCP_LOADCELL_CHANNEL);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setConversionMode(MCP356xMode::CONTINUOUS);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void setup() {
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(mcpScale);

  // MCP Calibration Configuration
  mcpScale.setScaleFactor(
      MCP_LOADCELL_CHANNEL, 
      0.0006093949987217794f);

  mcpScale.setLinearCalibration(
      MCP_LOADCELL_CHANNEL, 
      0.0006025509378972841f,
      -538.4288848531679f);

  mcpScale.setPolynomialCalibration(
      MCP_LOADCELL_CHANNEL, 
      -9.500760302035329e-14f, 
      0.0006027208452100216f,
      -538.3644732675577f);

  mcpScale.setConversionMode(MCP_LOADCELL_CHANNEL, MCP356xConversionMode::SINGLE_VALUE);
  mcpScale.tare(MCP_LOADCELL_CHANNEL);

  hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);

  // Set the single scaling factor
  hx711Scale.set_scale(
      0.0037841729238619852f
  );  

  hx711Scale.setLinearCalibration(
      0.003644352159139504f,
      511.07108437833887f
  );  

  hx711Scale.setPolynomialCalibration(
      -1.2109904581866693e-11f, 
      0.0036409638179757003f,
      511.3231515170753f
  );

  hx711Scale.setConversionMode(HX711::ConversionMode::SINGLE_VALUE);

  // Tare the scale
  hx711Scale.tare(10);
}

void loop() {
  bool printOutput = false;

  if (mcpScale.isr_fired && mcpScale.read() == 2) {
    mcpReading = mcpScale.getForce(MCP_LOADCELL_CHANNEL);
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
    unsigned long elapsedTime = micros() ; 
    char output[150];
    snprintf(output, sizeof(output), "%lu, %d, %d, %d, %d", 
            elapsedTime, 
            mcpReading,
            static_cast<int>(filteredMcpReading), 
            hx711Reading,
            static_cast<int>(filteredHx711Reading));

    Serial.println(output);
    Serial.flush();
  }

  //delayMicroseconds(90); // 
}
