/*
This example demonstrates using the MCP356x ADC library to read multiple 
differential input channels. The ADC converts analog voltages on 4 channels 
(DIFF_A, DIFF_B, DIFF_C, DIFF_D) into digital values.

The values are printed to the serial monitor in voltage format when the 'c'
key is pressed. Pressing other keys can print the register values, change 
the ADC configuration, etc.

An interrupt is triggered when a conversion is complete. The ISR is used
to read new ADC values as they become available.
*/

#include <SPI.h>
#include "MCP356x.h"

#define TEST_PROG_VERSION "v1.0"

/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
// Pins for the SPI connection to the ADC:
#define SDI_PIN             11
#define SDO_PIN             12
#define SCK_PIN             13
#define ADC_CS_PIN          2  // Chip select for ADC
#define ADC_IRQ_PIN         3  // Interrupt for ADC
#define MCLK_PIN            0  //

/*******************************************************************************
* Globals
*******************************************************************************/
// Variables that shape reporting...
bool     autoreport        = true;  // Dump live data to the console?
uint8_t  dev_selection     = 0;      // Start with the ADC selected.
uint8_t  disp_update_rate  = 5;      // Update in Hz for LED display
uint32_t disp_update_last  = 0;      // millis() when the display last updated.
uint32_t disp_update_next  = 0;      // millis() when the display next updates.

MCP356xConfig adc0Config = {ADC_IRQ_PIN, ADC_CS_PIN};
MCP356x       adc0(adc0Config);

/*******************************************************************************
* Functions to output things to the console
*******************************************************************************/

void printHelp() {
  StringBuilder output("\nMCP356x Example ");
  output.concat(TEST_PROG_VERSION);
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
  Serial.println((char*) output.string());
}


void setupADC(MCP356x& adc) {
  adc.setOption(MCP356X_FLAG_USE_INTERNAL_CLK);
  adc.init(&SPI);
  adc.setScanChannels(4,
    MCP356xChannel::DIFF_A,
    MCP356xChannel::DIFF_B,
    MCP356xChannel::DIFF_C,
    MCP356xChannel::DIFF_D);
  adc.setReferenceRange(-1.5f, 1.5f);
  adc.setOversamplingRatio(MCP356xOversamplingRatio::OSR_32);
  adc.setGain(MCP356xGain::GAIN_1);
  adc.setADCMode(MCP356xADCMode::ADC_CONVERSION_MODE);
}


/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial.print("\n\n");

  //analogWriteFrequency(14, 36621.09);

  SPI.setSCK(SCK_PIN);
  SPI.setMISO(SDO_PIN);
  SPI.setMOSI(SDI_PIN);
  SPI.begin();

  printHelp();
  disp_update_last = millis();
  disp_update_next = disp_update_last + 1000;

  setupADC(adc0);
}

/*******************************************************************************
* Main loop
********************************************************************************/
void loop() {
  int8_t ret = 0;
  StringBuilder output;
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '?':  printHelp();                      break;
      case 'a':  autoreport = !autoreport;         break;
      case 'r':  adc0.printRegs(&output);          break;
      case 'i':  adc0.printData(&output);          break;
      case 'c':  adc0.printChannelValues(&output, true); break;
      case 'p':  adc0.printPins(&output);          break;

      case 'U':
      case 'u':
        disp_update_rate += ('u' == c) ? -1 : 1;
        output.concatf("Display update rate is now %uHz\n", disp_update_rate);
        break;

      case '+':
      case '-':
        adc0.setCircuitSettleTime(adc0.getCircuitSettleTime() + (('+' == c) ? 1: -1));
        output.concatf("ADC dwell time is now %ums.\n", adc0.getCircuitSettleTime());
        break;

      case '[':
      case ']':
        {
          uint8_t osr = (uint8_t) adc0.getOversamplingRatio();
          ret = adc0.setOversamplingRatio((MCP356xOversamplingRatio) (osr + (('[' == c) ? -1:1)));
          output.concatf("Oversampling ratio is now %u.\n", (uint8_t) adc0.getOversamplingRatio());
        }
        break;

      case '{':
      case '}':
        {
          uint8_t gv = (uint8_t) adc0.getGain();
          ret = adc0.setGain((MCP356xGain) (gv + (('[' == c) ? -1:1)));
          output.concatf("Gain is now %u.\n", 1 << ((uint8_t) adc0.getGain()));
        }
        break;

      case 'R':
        ret = adc0.reset();
        output.concatf("reset() returns %d.\n", ret);
        break;
      case 'x':
        ret = adc0.refresh();
        output.concatf("refresh() returns %d.\n", ret);
        break;
      case 'I':
        ret = adc0.init();
        output.concatf("init() returns %d.\n", ret);
        break;
    }
  }
  else if (adc0.isr_fired) {
    if (2 == adc0.read()) {
      // The last of the whole set of requested channels was just read.
      if (autoreport) {
        if (disp_update_last <= millis()) {
          disp_update_last = millis() + 1000;
          adc0.printChannelValues(&output,true);
        }
      }
    }
  }


  // Dump any accumulated output to the serial port.
  if (output.length() > 0) { Serial.print((char*) output.string());  }
}