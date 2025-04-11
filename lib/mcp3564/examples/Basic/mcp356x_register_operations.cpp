/**
 * @file mcp356x_register_operations.cpp
 * @brief Interactive example for exploring MCP356x ADC register operations
 *
 * This example provides an interactive terminal interface for exploring the MCP356x ADC.
 * It allows viewing register contents, ADC information, pin configurations, and making
 * real-time adjustments to various ADC parameters including oversampling ratio, gain,
 * and circuit settling time. The interface provides menu-driven access through simple
 * one-key commands via the serial terminal.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - ADC_CS_PIN (7): Connect to MCP356x CS (Chip Select)
 * - ADC_IRQ_PIN (6): Connect to MCP356x IRQ (Interrupt Request)
 */

#include "Arduino.h"
#include "MCP356x.h"

#define PROGRAM_VERSION "v1.0"

/**
 * @brief Pin definitions and hardware constants
 */
// Pins for the SPI connection to the ADC
#define SDI_PIN      11
#define SDO_PIN      12
#define SCK_PIN      13
#define ADC_CS_PIN   7   // Chip select for ADC
#define ADC_IRQ_PIN  6   // Interrupt for ADC
#define MCLK_PIN     0   // Master clock pin

/**
 * @brief Global variables for user interface control
 */
// Variables that shape reporting
bool     autoReport = true;     // Automatically dump ADC data to console
uint8_t  devSelection = 0;      // Start with the ADC selected
uint8_t  dispUpdateRate = 5;    // Update rate in Hz for display
uint32_t dispUpdateLast = 0;    // Last display update timestamp
uint32_t dispUpdateNext = 0;    // Next scheduled display update

/**
 * @brief MCP356x ADC configuration structure
 */
MCP356xConfig config = {
    .irq_pin = ADC_IRQ_PIN,
    .cs_pin = ADC_CS_PIN,
    .mclk_pin = MCLK_PIN,
    .addr = 0x01,
    .spiInterface = &SPI,
    .numChannels = 4,
    .osr = MCP356xOversamplingRatio::OSR_32,
    .gain = MCP356xGain::GAIN_1,
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE
};

// Pointer for dynamic ADC creation
MCP356x* adc0 = nullptr;

/**
 * @brief Prints help information to the serial console
 * 
 * Displays a list of available commands and their descriptions to guide
 * users in interacting with the application.
 */
void printHelp() {
    StringBuilder output("\nMCP356x Example ");
    output.concat(PROGRAM_VERSION);
    output.concat("\nMeta:\n----------------------------\n");
    output.concat("?     This help output\n");
    output.concat("a     Toggle autoReport\n");
    output.concat("U/u   Increase/decrease autoReport rate (0 disables rate limiting)\n");

    output.concat("\nADC:\n----------------------------\n");
    output.concat("I     Initialize ADC\n");
    output.concat("R     Reset ADC\n");
    output.concat("x     Refresh ADC shadow registers\n");
    output.concat("p     Dump ADC pins\n");
    output.concat("r     Dump ADC registers\n");
    output.concat("i     ADC info\n");
    output.concat("c     Channel values\n");
    output.concat("t     Timing information\n");
    output.concat("-/+   Decrease/increase circuit settling time\n");
    output.concat("[/]   Decrease/increase oversampling ratio\n");
    output.concat("{/}   Decrease/increase gain\n");
    Serial.println((char*)output.string());
}

/**
 * @brief Setup function run once at startup
 * 
 * Initializes serial communication, SPI interface, and the MCP356x ADC.
 * Configures the ADC for scanning multiple differential channels.
 */
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.print("\n\n");

    // Initialize SPI interface
    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();

    // Initialize timing variables
    dispUpdateLast = millis();
    dispUpdateNext = dispUpdateLast + 1000;

    // Create and initialize the ADC
    adc0 = new MCP356x(config);

    // Display help information
    printHelp();

    // Configure the ADC to scan all four differential channels
    adc0->setScanChannels(4, 
        MCP356xChannel::DIFF_A, 
        MCP356xChannel::DIFF_B, 
        MCP356xChannel::DIFF_C, 
        MCP356xChannel::DIFF_D
    );
}

/**
 * @brief Main program loop
 * 
 * Handles serial input for interactive commands and manages automatic
 * reporting of ADC readings if enabled. Processes user commands to
 * modify ADC settings and display various information.
 */
void loop() {
    int8_t ret = 0;
    StringBuilder output;
    
    // Check for user input via serial
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            // Help and basic commands
            case '?':  
                printHelp();                      
                break;
            case 'a':  
                autoReport = !autoReport;         
                output.concatf("Autoreport is now %s\n", autoReport ? "ON" : "OFF");
                break;
            
            // ADC information commands
            case 'r':  
                adc0->printRegs(&output);          
                break;
            case 'i':  
                adc0->printData(&output);          
                break;
            case 'c':  
                adc0->printChannelValues(&output, true); 
                break;
            case 'p':  
                adc0->printPins(&output);          
                break;
            case 't': 
                adc0->printTimings(&output);        
                break;

            // Display update rate adjustment
            case 'U':
            case 'u':
                dispUpdateRate += ('u' == c) ? -1 : 1;
                if (dispUpdateRate < 0) dispUpdateRate = 0;
                output.concatf("Display update rate is now %u Hz\n", dispUpdateRate);
                break;

            // Circuit settling time adjustment
            case '+':
            case '-':
                adc0->setCircuitSettleTime(adc0->getCircuitSettleTime() + (('+' == c) ? 1 : -1));
                output.concatf("ADC dwell time is now %u ms\n", adc0->getCircuitSettleTime());
                break;

            // Oversampling ratio adjustment
            case '[':
            case ']':
                {
                    uint8_t osr = (uint8_t)adc0->getOversamplingRatio();
                    ret = adc0->setOversamplingRatio((MCP356xOversamplingRatio)(osr + (('[' == c) ? -1 : 1)));
                    output.concatf("Oversampling ratio is now %u\n", (uint8_t)adc0->getOversamplingRatio());
                }
                break;

            // Gain adjustment
            case '{':
            case '}':
                {
                    uint8_t gv = (uint8_t)adc0->getGain();
                    ret = adc0->setGain((MCP356xGain)(gv + (('{' == c) ? -1 : 1)));
                    output.concatf("Gain is now %u\n", 1 << ((uint8_t)adc0->getGain()));
                }
                break;

            // ADC reset and refresh commands
            case 'R':
                ret = adc0->reset();
                output.concatf("reset() returns %d\n", ret);
                break;
            case 'x':
                ret = adc0->refresh();
                output.concatf("refresh() returns %d\n", ret);
                break;
        }
    }
    // Process ADC readings if autoReport is enabled
    else if (adc0->updatedReadings()) {
        if (autoReport) {
            if (dispUpdateLast <= millis()) {
                dispUpdateLast = millis() + (dispUpdateRate > 0 ? (1000 / dispUpdateRate) : 1000);
                adc0->printChannelValues(&output, true);
            }
        }
    }

    // Output accumulated string to serial port
    if (output.length() > 0) { 
        Serial.print((char*)output.string()); 
    }
}