#include <Arduino.h>
#include "MCP356x.h"

// Pin definitions
#define SDI_PIN           11
#define SDO_PIN           12
#define SCK_PIN           13
#define ADC1_CS           2  
#define ADC1_IRQ          3  
#define ADC2_CS           4  
#define ADC2_IRQ          5  
#define ADC3_CS           7  
#define ADC3_IRQ          6  

// ADC objects
MCP356x adc1(ADC1_IRQ, ADC1_CS);
MCP356x adc2(ADC2_IRQ, ADC2_CS);
MCP356x adc3(ADC3_IRQ, ADC3_CS);

// Tracking variables
uint16_t readCount[3] = {0, 0, 0}; 
uint32_t lastReadTime[3] = {0, 0, 0};

void setupADC(MCP356x& adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(4,
    MCP356xChannel::DIFF_A,
    MCP356xChannel::DIFF_B,
    MCP356xChannel::DIFF_C,
    MCP356xChannel::DIFF_D);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setConversionMode(MCP356xMode::CONTINUOUS);
}

void readADC(MCP356x& adc, uint8_t adcNum) {
  if(adc.isrFired()) {
    adc.read(); 
    readCount[adcNum-1]++;
    if(readCount[adcNum-1] == 1000) {
      lastReadTime[adcNum-1] = micros();
      readCount[adcNum-1] = 0; 
    }
  }
}

enum ADCState {
    ADC1_STATE,
    ADC2_STATE,
    ADC3_STATE
};

ADCState currentState = ADC1_STATE;

void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");
  analogWriteFrequency(14, 36621.09);
  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();
  setupADC(adc1);
  setupADC(adc2);
  setupADC(adc3);
}

void loop() {
    if (adc1.isrFired() && adc2.isrFired() && adc3.isrFired()) {
        // Read all ADCs
        readADC(adc1, 1);
        readADC(adc2, 2);
        readADC(adc3, 3);
    }

    const unsigned long printInterval = 1000;
    static unsigned long lastPrintTime = 0;

    if (millis() - lastPrintTime >= printInterval) {
        float totalRate = 0;
        for (int i = 0; i < 3; i++) {
            float rate = 1000000.0 * readCount[i] / (micros() - lastReadTime[i]);
            totalRate += rate * 4;  // Since each ADC read results in 4 channel reads
            Serial.print("ADC"); 
            Serial.print(i+1);
            Serial.print(" rate: ");
            Serial.print(rate * 4);  // Display the rate considering 4 channels per ADC read
            Serial.println(" reads/sec");
            readCount[i] = 0;
            lastReadTime[i] = micros();
        }
        Serial.print("Overall sampling frequency: ");
        Serial.print(totalRate);
        Serial.println(" Hz");
        lastPrintTime = millis();
    }
}
