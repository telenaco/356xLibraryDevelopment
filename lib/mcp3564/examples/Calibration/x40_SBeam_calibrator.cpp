/**
 * @file main.cpp
 * @brief ADC Calibration for Four Channels
 * 
 * This program reads from four channels of an ADC which have not been calibrated.
 * Every 500 milliseconds, it will print the values.
 * The calibration process is done by inputting a string via the serial command with the 
 * name of the channel, A, B, C, or D, followed by the index of the weight 
 * being used, and the weight itself. For example: "a,1,300" for channel A, 
 * first weight, 300 grams.
 * Once calibration is complete for all channels and weights, inputting "X" 
 * will print the calibration data.
 * 
 * @author [Your Name]
 * @date [Your Date]
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>

/** @brief Pins Configuration */
const uint8_t SDI_PIN = 11;
const uint8_t SDO_PIN = 12;
const uint8_t SCK_PIN = 13;
const uint8_t MCP_ADC_CS_PIN = 2;
const uint8_t MCP_ADC_IRQ_PIN = 3;
const uint8_t MCLK_PIN = 0;
const uint8_t HX711_DOUT_PIN = 23;
const uint8_t HX711_SCK_PIN = 22;

/** @brief Pins Configuration */
const MCP356xChannel CHANNEL_A = MCP356xChannel::DIFF_A;
const MCP356xChannel CHANNEL_B = MCP356xChannel::DIFF_B;

/** @brief MCP356x ADC object */
MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);

const int NUM_CHANNELS = 2;  // Only A and B
struct MCPReadings {
    int A = 0;
    int B = 0;
} mcpReadings;

// Filter Configuration for MCP356x
constexpr double MCP_SAMPLING_FREQUENCY = 11000;
constexpr double MCP_CUT_OFF_FREQUENCY  = 48;
constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;

// Filter Configuration for MCP356x - Digital Reading
auto butterworthMcpDigitalFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);
// Filter Configuration for MCP356x - Force
auto butterworthMcpForceFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);

struct MCPFilteredReadings {
    float A = 0;
    float B = 0;
} mcpFilteredReadings;

struct MCPForces {
    float A = 0;
    float B = 0;
} mcpForces;

struct MCPFilteredForces {
    float A = 0;
    float B = 0;
} mcpFilteredForces;


void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(NUM_CHANNELS, CHANNEL_A, CHANNEL_B);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void printOutput() {
    unsigned long elapsedTime = micros();
    char output[200];
    snprintf(output, sizeof(output), "%lu, %d, %f, %f, %f", 
             elapsedTime, 
             mcpReadings.A,
             mcpFilteredReadings.A, 
             mcpForces.A,
             mcpFilteredForces.A);

    Serial.println(output);
    Serial.flush();
}

int channelToIndex(char channel) {
  switch (channel) {
    case 'a':
      return 0;
    case 'b':
      return 1;
    default:
      return -1;  // Invalid channel
  }
}

void setup() {
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(mcpScale);
  mcpScale.tare(CHANNEL_A);
  
  // Set Linear Calibration for S-beam load cell
  mcpScale.setLinearCalibration(CHANNEL_A, 0.0005683752659612749f, 17.224049032974072f);



  while (!Serial) {
    delay(10);
  }
  Serial.println("end of setup");
}

void loop() {
    if (mcpScale.isr_fired && mcpScale.read() == 2) {
        // Digital Readings
        mcpReadings.A = mcpScale.getDigitalValue(CHANNEL_A);
        // mcpReadings.B = mcpScale.getDigitalValue(CHANNEL_B);
        
        // Filtered Digital Readings
        mcpFilteredReadings.A = butterworthMcpDigitalFilter(mcpReadings.A);
        // mcpFilteredReadings.B = butterworthMcpDigitalFilter(mcpReadings.B);

        // Force Readings
        mcpForces.A = mcpScale.getForce(CHANNEL_A);
        // mcpForces.B = mcpScale.getForce(CHANNEL_B);
        
        // Filtered Force Readings
        mcpFilteredForces.A = butterworthMcpForceFilter(mcpForces.A);
        // mcpFilteredForces.B = butterworthMcpForceFilter(mcpForces.B);
    }

    // print every new reading
    printOutput();
}