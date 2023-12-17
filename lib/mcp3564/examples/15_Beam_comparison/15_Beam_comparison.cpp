#include "MCP356xScale.h"
#include "MCP356x.h"

/** @brief Pins Configuration */
const uint8_t SDI_PIN         = 11;
const uint8_t SDO_PIN         = 12;
const uint8_t SCK_PIN         = 13;

// Configuration pins for the ADC
MCP356xConfig adcConfig1 = {3, 2}; 
MCP356xScale mcpScale1(adcConfig1);

void printOutput() {
    unsigned long elapsedTime = micros();
    float         gramsForce  = mcpScale1.getGramsForce(MCP356xScale::A);
    char          output[20];
    snprintf(output, sizeof(output), "%lu, %.1f", elapsedTime, gramsForce);

    Serial.println(output);
    Serial.flush();
}

void setup() {
    Serial.begin(115200);

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // 4 channels, OSR_32
    mcpScale1.setupADC(&SPI, 4, MCP356xOversamplingRatio::OSR_32);  

    // Set polynomial calibration
    // mcpScale.setPolynomialCalibration(CHANNEL_A, -1.143e-13, 0.0003968, 261.8);    // Degraw calibration
    // mcpScale.setPolynomialCalibration(CHANNEL_A, -2.923e-13, 0.0005596, 513.9);     // TAL220 calibration
    mcpScale1.setPolynomialCalibration(MCP356xScale::A, -5.538e-14, 0.0003295, -227.8);   // CLZ635 calibration

    // Set the conversion mode to POLYNOMIAL
    mcpScale1.setConversionMode(MCP356xScale::A, MCP356xScale::ScaleConversionMode::POLYNOMIAL);
}

void loop() {
    if (mcpScale1.isr_fired && mcpScale1.read() == 2) {
        printOutput();
    }
}
