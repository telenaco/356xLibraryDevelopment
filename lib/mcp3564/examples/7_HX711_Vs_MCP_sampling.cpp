#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"

// MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define MCP_ADC_CS_PIN 2   // Chip select for MCP356x ADC
#define MCP_ADC_IRQ_PIN 3  // Interrupt for MCP356x ADC
#define MCLK_PIN 0         //

const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

// Define the configuration for the MCP356x
MCP356xConfig mcpConfig = {
    .irq_pin      = MCP_ADC_IRQ_PIN,                     // IRQ pin
    .cs_pin       = MCP_ADC_CS_PIN,                      // Chip select pin
    .mclk_pin     = MCLK_PIN,                            // MCLK pin
    .addr         = 0x01,                                // Device address (GND in this case)
    .spiInterface = &SPI,                                // SPI interface to use
    .numChannels  = 1,                                   // Number of channels to scan
    .osr          = MCP356xOversamplingRatio::OSR_32,    // Oversampling ratio
    .gain         = MCP356xGain::GAIN_1,                 // Gain setting
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE  // Continuous conversion mode
};

MCP356x* mcpScale = nullptr;

// HX711 Constants and Variables
const int HX711_DOUT_PIN = 23;
const int HX711_SCK_PIN = 22;

int mcpRawReading =0;
int hx711Reading= 0;
uint32_t lastSuccessfulMCPReadTime;
uint32_t lastSuccessfulHX711ReadTime;
uint32_t elapsedMCPMicros;
uint32_t elapsedHX711Micros;

HX711 hx711Scale;
volatile boolean hx711DataReady = false;

void hx711DataReadyISR() {
  if (hx711Scale.is_ready()) {
    hx711DataReady = true;
  }
}

void setupADC() {
    mcpScale->setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
    mcpScale->setScanChannels(1, MCP_LOADCELL_CHANNEL);
}

void setup() {
    Serial.begin(115200);
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    mcpScale = new MCP356x(mcpConfig);
    setupADC();

    hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    hx711Scale.tare(10);
    attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
}

void loop() {
    bool printOutput = false;  // Flag to determine if we should print the output
    int  mcpFlag     = 0;      // Flag to indicate if MCP has taken a reading in this loop
    int  hx711Flag   = 0;      // Flag to indicate if HX711 has taken a reading in this loop

    elapsedMCPMicros   = micros() - lastSuccessfulMCPReadTime;
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
          // Consistent-length output
        StringBuilder output;
        output.concatf("%d, %d, %lu, %d, %d", mcpRawReading, hx711Reading, millis(), mcpFlag, hx711Flag);
        Serial.println((char*)output.string());
    }

    // Reset the flags for the next loop iteration
    mcpFlag = 0;
    hx711Flag = 0;
}