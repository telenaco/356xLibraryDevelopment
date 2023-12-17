#include "MCP356xScale.h"
#include "MCP356x.h"

/** @brief Pins Configuration */
const uint8_t SDI_PIN         = 11;
const uint8_t SDO_PIN         = 12;
const uint8_t SCK_PIN         = 13;

// Configuration pins for the ADC
MCP356xConfig adcConfig1 = {3, 2}; 
MCP356xScale mcpScale1(adcConfig1);

/** Pins Configuration */
const MCP356xChannel CHANNEL_A = MCP356xChannel::DIFF_A;


const int NUM_CHANNELS = 1;
struct MCPData {
    int32_t A = 0;
};

MCPData mcpReadings;

void setupADC(MCP356xScale &adc) {

    adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    adc.init(&SPI);
    adc.setScanChannels(NUM_CHANNELS, CHANNEL_A);
    adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
    adc.setGain(MCP356xGain::GAIN_1);
    adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void printOutput() {
    unsigned long elapsedTime = micros();
    float         gramsForce  = mcpScale1.getGramsForce(CHANNEL_A);
    char          output[50];
    snprintf(output, sizeof(output), "%lu, %f", elapsedTime, gramsForce);

    Serial.println(output);
    Serial.flush();
}

void setup() {
    Serial.begin(115200);

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    setupADC(mcpScale1);

    // Set polynomial calibration for the appropiated beam load cell
    // mcpScale.setPolynomialCalibration(CHANNEL_A, -1.143e-13, 0.0003968, 261.8);    // Degraw calibration
    // mcpScale.setPolynomialCalibration(CHANNEL_A, -2.923e-13, 0.0005596, 513.9);     // TAL220 calibration
    mcpScale1.setPolynomialCalibration(CHANNEL_A, -5.538e-14, 0.0003295, -227.8);   // CLZ635 calibration

    // Set the conversion mode to POLYNOMIAL
    mcpScale1.setConversionMode(CHANNEL_A, MCP356xScale::ScaleConversionMode::POLYNOMIAL);
}

void loop() {

    if (mcpScale1.isr_fired && mcpScale1.read() == 2) {
        mcpReadings.A = mcpScale1.value(CHANNEL_A);

        printOutput();
    }
}
