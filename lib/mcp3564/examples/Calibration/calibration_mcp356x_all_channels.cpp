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
const MCP356xChannel CHANNEL_C = MCP356xChannel::DIFF_C;
const MCP356xChannel CHANNEL_D = MCP356xChannel::DIFF_D;

/** @brief MCP356x ADC object */
MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);


const int NUM_CHANNELS = 4;                          // a, b, c, d
struct MCPReadings {
    int A = 0;
    int B = 0;
    int C = 0;
    int D = 0;
} mcpReadings;

// Calibration data storage
float calibrationReadings[NUM_CHANNELS][9] = {{0}};  // Initialize all to 0
float calibrationWeights[NUM_CHANNELS][9] = {{0}};   // Initialize all to 0

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(NUM_CHANNELS, CHANNEL_A, CHANNEL_B, CHANNEL_C, CHANNEL_D);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void readMCPChannelsAverage() {

    // Declare and initialize channels to average
    MCP356xChannel channels[NUM_CHANNELS] = {CHANNEL_A, CHANNEL_B, CHANNEL_C, CHANNEL_D};
    
    // array to store the resulting averages
    int32_t averages[NUM_CHANNELS];

    // Get averages
    mcpScale.getMultipleChannelAverage(4000, channels, averages, NUM_CHANNELS);


    // Assign individual channel reading variables
    mcpReadings.A = averages[0];
    mcpReadings.B = averages[1];
    mcpReadings.C = averages[2];
    mcpReadings.D = averages[3];
}

void printOutput() {
  unsigned long elapsedTime = micros();
  char output[250];
  snprintf(output, sizeof(output), "%lu, %d, %d, %d, %d",
           elapsedTime, 
           mcpReadings.A, 
           mcpReadings.B,  
           mcpReadings.C,  
           mcpReadings.D);

  Serial.println(output);
  Serial.flush();
}

int channelToIndex(char channel) {
  switch (channel) {
    case 'a':
      return 0;
    case 'b':
      return 1;
    case 'c':
      return 2;
    case 'd':
      return 3;
    default:
      return -1;  // Invalid channel
  }
}

void storeCalibrationData(int channelIndex, int calibIndex, float weight,
                          float reading) {
  if (channelIndex >= 0 && channelIndex < NUM_CHANNELS && calibIndex >= 1 &&
      calibIndex <= 9) {
    calibrationReadings[channelIndex][calibIndex - 1] =
        reading;  // Adjust for 0-based index
    calibrationWeights[channelIndex][calibIndex - 1] = weight;
  }
}

void outputCalibrationData() {
  Serial.println("Channel,Index,Weight,Reading");
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < 9; i++) {
      char channelChar = 'a' + ch;  // Convert index back to channel char
      Serial.print(channelChar);
      Serial.print(",");
      Serial.print(i + 1);  // Adjust for 0-based index
      Serial.print(",");
      Serial.print(calibrationWeights[ch][i]);
      Serial.print(",");
      Serial.println(calibrationReadings[ch][i]);
    }
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    String inputData =
        Serial.readStringUntil('\n');  // Read until newline character
    inputData.trim();  // Remove any leading or trailing whitespace
    if (inputData == "X" || inputData == "x") {
      outputCalibrationData();
      return;
    }
    int firstSeparator = inputData.indexOf(
        ',');  // Find the first separator (comma) in the input
    int secondSeparator = inputData.indexOf(
        ',', firstSeparator + 1);  // Find the second separator
    if (firstSeparator != -1 && secondSeparator != -1) {
      char channelChar = inputData.charAt(0);
      int channelIndex = channelToIndex(channelChar);
      int calibIndex =
          inputData.substring(firstSeparator + 1, secondSeparator).toInt();
      float weight = inputData.substring(secondSeparator + 1).toFloat();
      float reading = 0;
      switch (channelChar) {
        case 'a':
          reading = mcpReadings.A;
          break;
        case 'b':
          reading = mcpReadings.B;
          break;
        case 'c':
          reading = mcpReadings.C;
          break;
        case 'd':
          reading = mcpReadings.D;
          break;
      }
      storeCalibrationData(channelIndex, calibIndex, weight, reading);
    }
  }
}

void setup() {
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(mcpScale);
  while (!Serial) {
    delay(10);
  }
  Serial.println("end of setup");
}


void loop() {
    // Read the average values from the channels
    readMCPChannelsAverage();
    // print every new reading
    printOutput();
    // Handle any input from the serial console
    handleSerialInput(); 
}