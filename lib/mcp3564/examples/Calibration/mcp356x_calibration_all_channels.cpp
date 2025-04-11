/**
 * @file mcp356x_calibration_all_channels.cpp
 * @brief Multi-channel ADC calibration for MCP356x.
 *
 * This program reads and calibrates four channels of an MCP356x ADC. It displays readings
 * every 500 milliseconds and supports interactive calibration through the serial port.
 * The user can input calibration commands in the format "channel,index,weight" (e.g., "a,1,300").
 * After completing the calibration process, sending "X" will output a complete calibration dataset.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - ADC_CS_PIN (2): Connect to MCP356x CS (Chip Select)
 * - ADC_IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 */

#include "MCP356x.h"

/** @brief Pins Configuration */
const uint8_t SDI_PIN = 11;
const uint8_t SDO_PIN = 12;
const uint8_t SCK_PIN = 13;
const uint8_t ADC_CS_PIN = 2;
const uint8_t ADC_IRQ_PIN = 3;
const uint8_t MCLK_PIN = 0;

/** @brief Channel Configuration */
const MCP356xChannel CHANNEL_A = MCP356xChannel::DIFF_A;
const MCP356xChannel CHANNEL_B = MCP356xChannel::DIFF_B;
const MCP356xChannel CHANNEL_C = MCP356xChannel::DIFF_C;
const MCP356xChannel CHANNEL_D = MCP356xChannel::DIFF_D;

/** @brief MCP356x ADC object */
MCP356x mcpScale(ADC_IRQ_PIN, ADC_CS_PIN, MCLK_PIN);

// Number of channels to monitor
const int NUM_CHANNELS = 4;  // a, b, c, d

/**
 * @brief Structure to store channel readings
 */
struct MCPReadings {
    int A = 0;
    int B = 0;
    int C = 0;
    int D = 0;
} mcpReadings;

// Calibration data storage
float calibrationReadings[NUM_CHANNELS][9] = {{0}};  // Initialize all to 0
float calibrationWeights[NUM_CHANNELS][9] = {{0}};   // Initialize all to 0

/**
 * @brief Configure the ADC for multi-channel operation
 * 
 * Sets up the ADC with appropriate settings for multi-channel calibration,
 * including internal clock, channel scanning, and conversion mode.
 * 
 * @param adc Reference to the MCP356x ADC object
 */
void setupADC(MCP356x &adc) {
    adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    adc.init(&SPI);
    adc.setScanChannels(NUM_CHANNELS, CHANNEL_A, CHANNEL_B, CHANNEL_C, CHANNEL_D);
    adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
    adc.setGain(MCP356xGain::GAIN_1);
    adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

/**
 * @brief Read and average multiple readings from all MCP channels
 * 
 * Takes multiple samples from each channel and calculates the average
 * to improve reading stability.
 */
void readMCPChannelsAverage() {
    // Declare and initialize channels to average
    MCP356xChannel channels[NUM_CHANNELS] = {CHANNEL_A, CHANNEL_B, CHANNEL_C, CHANNEL_D};
    
    // Array to store the resulting averages
    int32_t averages[NUM_CHANNELS];

    // Get averages
    mcpScale.getMultipleChannelAverage(4000, channels, averages, NUM_CHANNELS);

    // Assign individual channel reading variables
    mcpReadings.A = averages[0];
    mcpReadings.B = averages[1];
    mcpReadings.C = averages[2];
    mcpReadings.D = averages[3];
}

/**
 * @brief Format and print channel readings to serial port
 * 
 * Outputs timestamp and readings for all channels.
 */
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

/**
 * @brief Convert channel character to index
 * 
 * Helper function to convert a channel identifier character to its
 * corresponding index for use in arrays.
 * 
 * @param channel Channel identifier ('a' through 'd')
 * @return int Index of the channel, or -1 if invalid
 */
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

/**
 * @brief Store calibration data for a specific channel and calibration point
 * 
 * @param channelIndex Index of the channel (0-3)
 * @param calibIndex Index of the calibration point (1-9)
 * @param weight Known weight value for calibration
 * @param reading ADC reading for the given weight
 */
void storeCalibrationData(int channelIndex, int calibIndex, float weight,
                          float reading) {
    if (channelIndex >= 0 && channelIndex < NUM_CHANNELS && calibIndex >= 1 &&
        calibIndex <= 9) {
        calibrationReadings[channelIndex][calibIndex - 1] = reading;  // Adjust for 0-based index
        calibrationWeights[channelIndex][calibIndex - 1] = weight;
    }
}

/**
 * @brief Output complete calibration dataset
 * 
 * Prints all stored calibration data points to the serial port
 * in a CSV format: Channel,Index,Weight,Reading
 */
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

/**
 * @brief Handle serial input for calibration commands
 * 
 * Process user input for setting calibration points or outputting
 * calibration data. Format: "channel,index,weight" or "X" to output.
 */
void handleSerialInput() {
    while (Serial.available()) {
        String inputData = Serial.readStringUntil('\n');  // Read until newline character
        inputData.trim();  // Remove any leading or trailing whitespace
        
        // Check for export command
        if (inputData == "X" || inputData == "x") {
            outputCalibrationData();
            return;
        }
        
        // Parse calibration command
        int firstSeparator = inputData.indexOf(',');  // Find the first separator (comma)
        int secondSeparator = inputData.indexOf(',', firstSeparator + 1);  // Find the second separator
        
        if (firstSeparator != -1 && secondSeparator != -1) {
            char channelChar = inputData.charAt(0);
            int channelIndex = channelToIndex(channelChar);
            int calibIndex = inputData.substring(firstSeparator + 1, secondSeparator).toInt();
            float weight = inputData.substring(secondSeparator + 1).toFloat();
            float reading = 0;
            
            // Get the reading for the specified channel
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
            
            // Store the calibration data point
            storeCalibrationData(channelIndex, calibIndex, weight, reading);
        }
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Configure ADC for multi-channel calibration
    setupADC(mcpScale);
    
    // Wait for serial connection before proceeding
    while (!Serial) {
        delay(10);
    }
    Serial.println("Setup complete - multi-channel calibration ready");
}

/**
 * @brief Main program loop
 * 
 * Continuously reads from the ADC channels, outputs readings,
 * and processes any calibration commands received via serial.
 */
void loop() {
    // Read the average values from the channels
    readMCPChannelsAverage();
    
    // Print every new reading
    printOutput();
    
    // Handle any input from the serial console
    handleSerialInput(); 
}