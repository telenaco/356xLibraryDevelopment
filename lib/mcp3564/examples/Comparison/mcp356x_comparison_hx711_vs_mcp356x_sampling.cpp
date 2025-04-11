/**
 * @file mcp356x_comparison_hx711_vs_mcp356x_sampling.cpp
 * @brief Compares the sampling performance of MCP356x ADC and HX711 ADC.
 *
 * This program simultaneously reads data from an MCP356x ADC and an HX711 ADC,
 * and compares their sampling performance. It measures the elapsed time between
 * successful readings for each ADC and outputs the raw readings, elapsed times,
 * and flags indicating which ADC has taken a reading in each loop iteration.
 * This allows for direct performance comparison between these two ADC types
 * under identical conditions.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - MCP_CS_PIN (2): Connect to MCP356x CS (Chip Select)
 * - MCP_IRQ_PIN (3): Connect to MCP356x IRQ (Interrupt Request)
 * - HX711_DOUT_PIN (23): Connect to HX711 DOUT (Data Output)
 * - HX711_SCK_PIN (22): Connect to HX711 SCK (Serial Clock Input)
 */

#include "MCP356x.h"
#include <Filters.h>
#include <Filters/Butterworth.hpp>
#include "HX711.h"

// Pin definitions for SPI and ADC connections
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define MCP_CS_PIN 2   // Chip select for MCP356x ADC
#define MCP_IRQ_PIN 3  // Interrupt for MCP356x ADC

// HX711 connection pins
#define HX711_DOUT_PIN 23
#define HX711_SCK_PIN 22

// ADC channel configuration
const MCP356xChannel MCP_LOADCELL_CHANNEL = MCP356xChannel::DIFF_A;

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig mcpConfig = {
    .irq_pin      = MCP_IRQ_PIN,                    // IRQ pin
    .cs_pin       = MCP_CS_PIN,                     // Chip select pin
    .mclk_pin     = 0,                              // MCLK pin (0 for internal clock)
    .addr         = 0x01,                           // Device address (GND in this case)
    .spiInterface = &SPI,                           // SPI interface to use
    .numChannels  = 1,                              // Number of channels to scan
    .osr          = MCP356xOversamplingRatio::OSR_32, // Oversampling ratio
    .gain         = MCP356xGain::GAIN_1,            // Gain setting (1x)
    .mode         = MCP356xADCMode::ADC_CONVERSION_MODE // Continuous conversion mode
};

// ADC objects
MCP356x* mcpScale = nullptr;
HX711 hx711Scale;

// Data ready flag for HX711
volatile boolean hx711DataReady = false;

// Variables for storing readings and timing information
int mcpRawReading = 0;
int hx711Reading  = 0;
uint32_t lastSuccessfulMCPReadTime;
uint32_t lastSuccessfulHX711ReadTime;
uint32_t elapsedMCPMicros;
uint32_t elapsedHX711Micros;

/**
 * @brief Interrupt service routine for HX711 data ready event
 * 
 * Sets the hx711DataReady flag when new data is available from the HX711 ADC.
 */
void hx711DataReadyISR() {
    if (hx711Scale.is_ready()) {
        hx711DataReady = true;
    }
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, the MCP356x ADC,
 * and the HX711 ADC with interrupt handling for data ready detection.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Initialize MCP356x ADC
    mcpScale = new MCP356x(mcpConfig);

    // Initialize HX711 ADC
    hx711Scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
    hx711Scale.tare(10);
    
    // Attach interrupt for HX711 data ready detection
    attachInterrupt(digitalPinToInterrupt(HX711_DOUT_PIN), hx711DataReadyISR, FALLING);
    
    // Print CSV header
    Serial.println("MCP Reading, HX711 Reading, Timestamp (ms), MCP Flag, HX711 Flag");
}

/**
 * @brief Main program loop
 * 
 * Continuously checks for new data from both ADCs, measures the time
 * between successful readings, and outputs comparison data in CSV format.
 */
void loop() {
    bool printOutput = false;
    int  mcpFlag = 0;
    int  hx711Flag = 0;

    // Calculate elapsed time since last successful readings
    elapsedMCPMicros = micros() - lastSuccessfulMCPReadTime;
    elapsedHX711Micros = micros() - lastSuccessfulHX711ReadTime;

    // Check for new MCP356x readings
    if (mcpScale->updatedReadings()) {
        elapsedMCPMicros          = micros() - lastSuccessfulMCPReadTime;
        mcpRawReading             = mcpScale->value(MCP_LOADCELL_CHANNEL);
        lastSuccessfulMCPReadTime = micros();
        printOutput               = true;
        mcpFlag                   = 1;
    }

    // Check for new HX711 readings
    if (hx711DataReady) {
        elapsedHX711Micros          = micros() - lastSuccessfulHX711ReadTime;
        hx711Reading                = hx711Scale.read();
        hx711DataReady              = false;
        lastSuccessfulHX711ReadTime = micros();
        printOutput                 = true;
        hx711Flag                   = 1;
    }

    // Output the readings and flags if new data is available from either ADC
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

    // Reset flags for the next iteration
    mcpFlag = 0;
    hx711Flag = 0;
}