#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <StringBuilder.h>
#include "MCP356x.h"

// Pin definitions

#define SDI_PIN           11
#define SDO_PIN           12
#define SCK_PIN           13

#define ADC0_CS         2  // Chip select for ADC
#define ADC0_IRQ        3  // Interrupt for ADC
#define ADC1_CS          4  // Chip select for ADC
#define ADC1_IRQ         5  // Interrupt for ADC
#define ADC2_CS          6  // Chip select for ADC
#define ADC2_IRQ         7  // Interrupt for ADC

// ADC objects
MCP356x adc1(ADC0_IRQ, ADC0_CS, 0); // Slot 0
MCP356x adc2(ADC1_IRQ, ADC1_CS, 0); // Slot 1  
MCP356x adc3(ADC2_IRQ, ADC2_CS, 0); // Slot 2

// Variables for reporting  
bool autoreport = true;
uint8_t disp_update_rate = 50;
uint32_t disp_update_last = 0;
uint32_t disp_update_next = 0;

void printHelp() {
  StringBuilder output("\nMCP356x Example ");
  output.concat("\nMeta:\n----------------------------\n");
  output.concat("?     This output\n");
  output.concat("a     Toggle autoreport\n");
  output.concat("U/u   Autoreport rate up/down (0 disables rate limiting)\n");

  output.concat("\nADC:\n----------------------------\n");
  output.concat("I     Initialize ADC\n");
  output.concat("R     Reset ADC\n");
  output.concat("x     Refresh ADC shadow registers\n");
  output.concat("p     Dump ADC pins\n");
  output.concat("r     Dump ADC registers\n");
  output.concat("i     ADC info\n");
  output.concat("-/+   (De/In)crease circuit settling time\n");
  output.concat("[/]   (De/In)crease oversampling ratio.\n");
  output.concat("{/}   (De/In)crease gain.\n");

  Serial.println((char*)output.string());
}

void setupADC(MCP356x& adc) {
  adc.init(&SPI);
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.setScanChannels(4,
    MCP356xChannel::DIFF_A,
    MCP356xChannel::DIFF_B,
    MCP356xChannel::DIFF_C,
    MCP356xChannel::DIFF_D);
  adc.useInternalVref(true);
  adc.setReferenceRange(3.3f, 0.0f);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_64);
  //adc.setConversionMode(MCP356xMode::CONTINUOUS);
  adc.setGain(MCP356xGain::GAIN_1);
  //adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}

void handleADCRead(MCP356x& adc, StringBuilder& output, uint8_t adcNumber) {
  if (adc.isr_fired) {
    if (2 == adc.read()) {
      if (autoreport && disp_update_last <= millis()) {
        output.concatf("ADC %d Values:\n", adcNumber); // Print the ADC number
        adc.printChannelValues(&output, true);
        Serial.println("--------------------------------------------------");
        //adc.printChannelValues(&output, false);
        disp_update_last = millis() + 1000;  // Update the last update timestamp here.
      }
    }
  }
}

void handleSerialCommands(MCP356x& adc, StringBuilder& output, uint8_t adcNumber) {
  int8_t ret = 0;
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
    case '?':  printHelp();                             break;
    case 'a':  autoreport = !autoreport;                break;
    case 'r':  adc.printRegs(&output);                  break;
    case 'i':  adc.printData(&output);                  break;
    case 'c':  adc.printChannelValues(&output, true);   break;
    case 'p':  adc.printPins(&output);                  break;

    case 'U':
    case 'u':
      disp_update_rate += ('u' == c) ? -10 : 10;
      output.concatf("Display update rate is now %uHz\n", disp_update_rate);
      break;

    case '+':
    case '-':
      adc.setCircuitSettleTime(adc.getCircuitSettleTime() + (('+' == c) ? 1 : -1));
      output.concatf("ADC%d dwell time is now %ums.\n", adcNumber, adc.getCircuitSettleTime());
      break;

    case '[':
    case ']':
    {
      uint8_t osr = (uint8_t)adc.getOversamplingRatio();
      ret = adc.setOversamplingRatio((MCP356xOversamplingRatio)(osr + (('[' == c) ? -10 : 10)));
      output.concatf("ADC%d Oversampling ratio is now %u.\n", adcNumber, (uint8_t)adc.getOversamplingRatio());
    }
    break;

    case '{':
    case '}':
    {
      uint8_t gv = (uint8_t)adc.getGain();
      ret = adc.setGain((MCP356xGain)(gv + (('[' == c) ? -1 : 1)));
      output.concatf("ADC%d Gain is now %u.\n", adcNumber, 1 << ((uint8_t)adc.getGain()));
    }
    break;

    case 'R':
      ret = adc.reset();
      output.concatf("ADC%d reset() returns %d.\n", adcNumber, ret);
      break;
    case 'x':
      ret = adc.refresh();
      output.concatf("ADC%d refresh() returns %d.\n", adcNumber, ret);
      break;
    case 'I':
      ret = adc.init();
      output.concatf("ADC%d init() returns %d.\n", adcNumber, ret);
      break;
    }
  }
}


/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {

  Serial.begin(115200);
  Serial.print("\n\n");

  analogWriteFrequency(14, 36621.09);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  printHelp();
  disp_update_last = millis();
  disp_update_next = disp_update_last + 1000;

  SPI.begin();

  // Setup ADCs
  setupADC(adc1);
  setupADC(adc2);
  setupADC(adc3);

  Serial.println("end of setup");
}


/*******************************************************************************
* Main loop
********************************************************************************/
void loop() {
  StringBuilder output;

  //handleSerialCommands(adc1, output, 1);  // For ADC3


  // Handle ADC reads
  handleADCRead(adc1, output, 1);  // For ADC1
  handleADCRead(adc2, output, 2);  // For ADC2
  handleADCRead(adc3, output, 3);  // For ADC3

  // Dump any accumulated output to the serial port.
  if (output.length() > 0) { Serial.print((char*)output.string()); }
}