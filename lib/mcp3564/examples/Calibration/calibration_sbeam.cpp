/**
 * @file calibration_sbeam.cpp
 * @brief MCP356x ADC calibration for S-beam load cell with filtering.
 *
 * This program configures a MCP356x ADC for reading from an S-beam load cell with multiple
 * data processing paths. It captures and displays raw digital readings, Butterworth filtered
 * readings, calibrated force readings, and filtered force readings. The program also supports
 * serial commands for tare operations, making it suitable for calibration and verification.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - MCP_ADC_CS_PIN (2): Connect to MCP356x CS (Chip Select)
 * - MCP_ADC_IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 * - MCLK_PIN (0): Connect to MCP356x MCLK (Master Clock) - optional
 */

 #include "MCP356x.h"
 #include <Filters.h>
 #include <Filters/Butterworth.hpp>
 
 // Pin definitions for SPI communication
 #define SDI_PIN 11
 #define SDO_PIN 12
 #define SCK_PIN 13
 #define MCP_ADC_CS_PIN 2
 #define MCP_ADC_IRQ_PIN 3
 #define MCLK_PIN 0
 
 // ADC channel configuration
 #define CHANNEL_A MCP356xChannel::DIFF_A
 #define CHANNEL_B MCP356xChannel::DIFF_B
 
 /**
  * @brief MCP356x ADC object
  */
 MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);
 
 // Number of channels to monitor
 const int NUM_CHANNELS = 2;  // Only A and B
 
 /**
  * @brief Structure to store raw digital readings
  */
 struct MCPReadings {
     int A = 0;
     int B = 0;
 } mcpReadings;
 
 // Filter Configuration for MCP356x
 constexpr double MCP_SAMPLING_FREQUENCY = 11000;
 constexpr double MCP_CUT_OFF_FREQUENCY  = 48;
 constexpr double MCP_NORMALIZED_CUT_OFF = 2 * MCP_CUT_OFF_FREQUENCY / MCP_SAMPLING_FREQUENCY;
 
 // Filter instances for different data streams
 auto butterworthMcpDigitalFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);  // For digital readings
 auto butterworthMcpForceFilter = butter<1>(MCP_NORMALIZED_CUT_OFF);    // For force readings
 
 /**
  * @brief Structure to store filtered digital readings
  */
 struct MCPFilteredReadings {
     float A = 0;
     float B = 0;
 } mcpFilteredReadings;
 
 /**
  * @brief Structure to store calibrated force values
  */
 struct MCPForces {
     float A = 0;
     float B = 0;
 } mcpForces;
 
 /**
  * @brief Structure to store filtered force values
  */
 struct MCPFilteredForces {
     float A = 0;
     float B = 0;
 } mcpFilteredForces;
 
 /**
  * @brief Configure the ADC with appropriate settings
  * 
  * Sets up the ADC with internal clock, scan channels, oversampling ratio,
  * and gain settings for optimal load cell reading.
  * 
  * @param adc Reference to the MCP356x ADC object
  */
 void setupADC(MCP356x &adc) {
     adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
     adc.init(&SPI);
     adc.setScanChannels(NUM_CHANNELS, CHANNEL_A, CHANNEL_B);
     adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
     adc.setGain(MCP356xGain::GAIN_1);
     adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
 }
 
 /**
  * @brief Format and print readings to serial port
  * 
  * Outputs timestamp, raw reading, filtered reading, force, and filtered force values.
  */
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
 
 /**
  * @brief Convert channel character to index
  * 
  * Helper function to convert a channel identifier character to its
  * corresponding index for use in arrays.
  * 
  * @param channel Channel identifier ('a' or 'b')
  * @return int Index of the channel, or -1 if invalid
  */
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
 
 /**
  * @brief Setup function run once at startup
  * 
  * Initializes serial communication, SPI interface, and the MCP356x ADC.
  * Configures the ADC and applies initial calibration for the S-beam load cell.
  */
 void setup() {
     // Initialize serial communication
     Serial.begin(115200);
 
     // Initialize SPI interface
     SPI.setSCK(SCK_PIN);
     SPI.setMISO(SDO_PIN);
     SPI.setMOSI(SDI_PIN);
     SPI.begin();
 
     // Configure ADC for S-beam load cell
     setupADC(mcpScale);
     mcpScale.tare(CHANNEL_A);
     
     // Set Linear Calibration for S-beam load cell
     mcpScale.setLinearCalibration(CHANNEL_A, 0.0005683752659612749f, 17.224049032974072f);
 
     // Wait for serial connection before proceeding
     while (!Serial) {
         delay(10);
     }
     Serial.println("Setup complete - S-beam calibration ready");
 }
 
 /**
  * @brief Main program loop
  * 
  * Continuously reads from the ADC, applies filtering, and processes
  * the data through multiple paths (raw, filtered, calibrated, etc.).
  * Outputs processed readings to the serial port.
  */
 void loop() {
     // Check for new ADC data
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
 
     // Output the readings to serial port
     printOutput();
 }