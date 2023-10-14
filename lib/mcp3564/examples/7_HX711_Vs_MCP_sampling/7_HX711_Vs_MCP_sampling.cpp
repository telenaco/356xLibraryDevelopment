#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"

// MCP356x Constants
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define MCP_ADC_CS_PIN 2  // Chip select for MCP356x ADC
#define MCP_ADC_IRQ_PIN 3 // Interrupt for MCP356x ADC
#define MCLK_PIN 0

const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;
MCP356x mcpScale(MCP_ADC_IRQ_PIN, MCP_ADC_CS_PIN, MCLK_PIN);


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

void setupADC(MCP356x &adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(1, MCP_LOADCELL_CHANNEL);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void setup() {
  Serial.begin(115200);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  setupADC(mcpScale);
  mcpScale.tare(MCP_LOADCELL_CHANNEL);
  
  hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  hx711Scale.tare(10);
  attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
}

void loop() {
    bool printOutput = false; // Flag to determine if we should print the output
    int mcpFlag = 0;  // Flag to indicate if MCP has taken a reading in this loop
    int hx711Flag = 0; // Flag to indicate if HX711 has taken a reading in this loop

    elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
    elapsedHX711Micros = micros() - lastSuccessfulHX711ReadTime;

    if (mcpScale.isr_fired && mcpScale.read() == 2) {
        elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
        mcpRawReading = mcpScale.getDigitalValue(MCP_LOADCELL_CHANNEL);
        lastSuccessfulMCPReadTime = micros();
        printOutput = true;
        mcpFlag = 1;
    }

    if (hx711DataReady) {
        elapsedHX711Micros = micros() - lastSuccessfulHX711ReadTime;
        hx711Reading = hx711Scale.read();
        hx711DataReady = false;
        lastSuccessfulHX711ReadTime = micros();
        printOutput = true;
        hx711Flag = 1;
    }

    if(printOutput) {
        // Consistent-length output
        char output[130]; // Increased size to accommodate the flags
        snprintf(output, sizeof(output), "%d, %d, %lu, %d, %d", 
                mcpRawReading, 
                hx711Reading, 
                millis(),                
                mcpFlag,
                hx711Flag);

        Serial.println(output);
        Serial.flush();
    }

    // Reset the flags for the next loop iteration
    mcpFlag = 0;
    hx711Flag = 0;
    delayMicroseconds(90);
}