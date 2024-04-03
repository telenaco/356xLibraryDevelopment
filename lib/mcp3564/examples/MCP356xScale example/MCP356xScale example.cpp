#include "Arduino.h"
#include "MCP356xScale.h"

#define SCK_PIN 13
#define SDO_PIN 12
#define SDI_PIN 11
#define IRQ_PIN_0 3
#define CS_PIN_0 2

// Total scales represents the number of scales (or channels) you are using
int totalScales = 1;

// Create an instance of MCP356xScale
MCP356xScale scale(totalScales, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN_0, CS_PIN_0);

void setup() {
  Serial.begin(9600);
  // Optionally, here you can set up channel mappings, perform calibration, and set scale factors
}

void loop() {
  // Update readings from all ADCs
  scale.updatedForceReadings();
  
  // For demonstration, assuming we are working with the first scale (index 0)
  int scaleIndex = 0;

  // Get the force reading from the specified scale index
  float force = scale.getForce(scaleIndex);

  // Print the force reading
  Serial.print("Force: ");
  Serial.println(force);

  delay(1000); // Delay to make the output readable
}


#include "Arduino.h"
#include "MCP356x.h"

#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Defautl configuration for the MCP356x ADC pins are later update before instantiating them. 
MCP356xConfig config = {
    .irq_pin = 6,                                                            // IRQ pin
    .cs_pin = 7,                                                             // Chip select pin
    .mclk_pin = 0,                                                           // MCLK pin
    .addr = 0x01,                                                            // Device address (GND in this case)
    .spiInterface = &SPI,                                                    // SPI interface to use
    .numChannels = 4,                                                        // Number of channels to scan
    .osr = MCP356xOversamplingRatio::OSR_32,                                 // Oversampling ratio
    .gain = MCP356xGain::GAIN_1,                                             // Gain setting
    .mode = MCP356xADCMode::ADC_CONVERSION_MODE                              // Continuous conversion mode
};



                                                                             // Pointers for dynamic creation
MCP356x* adc1 = nullptr;
MCP356x* adc2 = nullptr;
MCP356x* adc3 = nullptr;

void readAndPrintADC(MCP356x* adc, const char* adcName) {
    if (adc->read()) {
                                                                             // Array of differential channels
        MCP356xChannel diffChannels[] = { MCP356xChannel::DIFF_A, MCP356xChannel::DIFF_B, MCP356xChannel::DIFF_C, MCP356xChannel::DIFF_D };
        char text[350];                                                      // Buffer for formatted output

        for (int i = 0; i < 4; ++i) {                                        // Iterate through the 4 differential channels
            int32_t adc_value = adc->value(diffChannels[i]);
                                                                             // Format and print each channel's value
            snprintf(text, sizeof(text), "%s Channel %d (DIFF_%c): %ld", adcName, i, 'A' + i, adc_value);
            Serial.println(text);
        }
    }
}


void setup() {

    Serial.begin(2000000);

    SPI.setSCK(SCK_PIN);
    SPI.setMISO(SDO_PIN);
    SPI.setMOSI(SDI_PIN);
    SPI.begin();
                                                        
    adc1 = new MCP356x(config);                                             // Initialize each MCP356x ADC    
    config.cs_pin = 4;
    config.irq_pin = 5;
    adc2 = new MCP356x(config);
    config.cs_pin = 2;
    config.irq_pin = 3;
    adc3 = new MCP356x(config);
}

void loop() {
    static unsigned long lastReadTime = 0;                                   // Stores the last time readings were taken
    unsigned long currentTime = micros();

    // Check if adc0's interrupt flag is fired and all ADCs have new data and are ready                                                    
    if ((2 == adc1->read()) && (2 == adc2->read()) && (2 == adc3->read())) {

                                                                             // Capture start time of reading
        unsigned long startTime = micros();

                                                                             // Prepare buffer and read voltages from all ADCs
        char buffer[512];                                                    // Ensure the buffer is large enough

        snprintf(buffer, sizeof(buffer),
            "ADC1: [%lu, %lu, %lu, %lu], ADC2: [%lu, %lu, %lu, %lu], ADC3: [%lu, %lu, %lu, %lu]",
            adc1->value(MCP356xChannel::DIFF_A), adc1->value(MCP356xChannel::DIFF_B),
            adc1->value(MCP356xChannel::DIFF_C), adc1->value(MCP356xChannel::DIFF_D),
            adc2->value(MCP356xChannel::DIFF_A), adc2->value(MCP356xChannel::DIFF_B),
            adc2->value(MCP356xChannel::DIFF_C), adc2->value(MCP356xChannel::DIFF_D),
            adc3->value(MCP356xChannel::DIFF_A), adc3->value(MCP356xChannel::DIFF_B),
            adc3->value(MCP356xChannel::DIFF_C), adc3->value(MCP356xChannel::DIFF_D));
                                                        
        Serial.println(buffer);                                              // Print combined readings
                                                        
        unsigned long endTime = micros();                                    // Print the time taken for the readings
        Serial.print("Timesss for readings: ");
        Serial.print(endTime - startTime);
        Serial.println(" microseconds");
                                                        
        Serial.print("Time since last readings: ");                          // Print the time since the last readings were taken
        Serial.print(startTime - lastReadTime);
        Serial.println(" microseconds");
                                                        
        lastReadTime = startTime;                                            // Update lastReadTime
    }
}
