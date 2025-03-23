/**
* @file 7_HX711_Vs_MCP_sampling.cpp
* @brief Compares the sampling performance of MCP356x ADC and HX711 ADC.
*
* This script simultaneously reads data from an MCP356x ADC and an HX711 ADC,
* and compares their sampling performance. It measures the elapsed time between
* successful readings for each ADC and prints the raw readings, elapsed times,
* and flags indicating which ADC has taken a reading in each loop iteration.
*
* Hardware Connections:
* - SDI_PIN           : Connect to MCP356x SDI (Serial Data Input).
* - SDO_PIN           : Connect to MCP356x SDO (Serial Data Output).
* - SCK_PIN           : Connect to MCP356x SCK (Serial Clock Input) and HX711 SCK.
* - MCP_ADC_IRQ_PIN   : Connect to MCP356x IRQ (Interrupt Request).
* - MCP_ADC_CS_PIN    : Connect to MCP356x CS (Chip Select).
* - HX711_DOUT_PIN    : Connect to HX711 DOUT (Data Output).
*/

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define MCP_ADC_IRQ_PIN 6
#define MCP_ADC_CS_PIN  7
#define HX711_DOUT_PIN 23
#define HX711_SCK_PIN  22  //

const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

// Define the configuration for the MCP356x
MCP356xConfig mcpConfig = {
    .irq_pin      = MCP_ADC_IRQ_PIN,                     // IRQ pin
    .cs_pin       = MCP_ADC_CS_PIN,                      // Chip select pin
    .mclk_pin     = 0,                                   // MCLK pin
    .addr         = 0x01,                                // Device address (GND in this case)
    .spiInterface = &SPI,                                // SPI interface to use
    .numChannels  = 1,                                   // Number of channels to scan
    .osr          = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain         = MCP356xGain::GAIN_1,                 // Gain setting
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

MCP356x* mcpScale = nullptr;
HX711 hx711Scale;

volatile boolean hx711DataReady = false;

int mcpRawReading = 0;
int hx711Reading  = 0;
uint32_t lastSuccessfulMCPReadTime;
uint32_t lastSuccessfulHX711ReadTime;
uint32_t elapsedMCPMicros;
uint32_t elapsedHX711Micros;

/**
* @brief Interrupt service routine for HX711 data ready event.
*
* This function is called when the HX711 ADC has new data available.
* It sets the hx711DataReady flag to true.
*/
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

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    mcpScale = new MCP356x(mcpConfig);

    hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    hx711Scale.tare(10);
    attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
}

void loop() {
    bool printOutput = false;
    int  mcpFlag = 0;
    int  hx711Flag = 0;

    elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
    elapsedHX711Micros = micros() - lastSuccessfulHX711ReadTime;

    if (mcpScale->updatedReadings()) {
        elapsedMCPMicros          = micros() - lastSuccessfulMCPReadTime;
        mcpRawReading             = mcpScale->value(MCP_LOADCELL_CHANNEL);
        lastSuccessfulMCPReadTime = micros();
        printOutput               = true;
        mcpFlag                   = 1;
    }

    if (hx711DataReady) {
        elapsedHX711Micros          = micros() - lastSuccessfulHX711ReadTime;
        hx711Reading                = hx711Scale.read();
        hx711DataReady              = false;
        lastSuccessfulHX711ReadTime = micros();
        printOutput                 = true;
        hx711Flag                   = 1;
    }

    if (printOutput) {
        StringBuilder output;
        output.concatf("%d, %d, %lu, %d, %d",
            mcpRawReading,
            hx711Reading,
            millis(),
            mcpFlag,
            hx711Flag);

        Serial.println((char*)output.string());
    }

    mcpFlag = 0;
    hx711Flag = 0;
}