/**
 * @file mcp356x_telemetry_12_cells.cpp
 * @brief This program reads data from 12 load cells using the MCP356xScale library and sends the average readings
 *        in a format compatible with the Telemetry Viewer.
 *
 * The program continuously updates the ADC readings from the load cells and calculates the average readings
 * over a specified sampling duration. The average readings are then sent as a comma-separated string via the
 * serial port. The program also supports a tare command to perform tare calibration with a specified number
 * of iterations.
 *
 * @author Jose Luis Berna Moya
 * @date Last Updated: March 2025
 * @version 1.0
 *
 * Hardware Connections:
 * - SDI_PIN (11): Connect to MCP356x SDI (Serial Data Input)
 * - SDO_PIN (12): Connect to MCP356x SDO (Serial Data Output)
 * - SCK_PIN (13): Connect to MCP356x SCK (Serial Clock Input)
 * - CS_PIN0 (7): Connect to first MCP356x CS (Chip Select)
 * - IRQ_PIN0 (6): Connect to first MCP356x IRQ (Interrupt Request)
 * - CS_PIN1 (4): Connect to second MCP356x CS (Chip Select)
 * - IRQ_PIN1 (5): Connect to second MCP356x IRQ (Interrupt Request)
 * - CS_PIN2 (2): Connect to third MCP356x CS (Chip Select)
 * - IRQ_PIN2 (3): Connect to third MCP356x IRQ (Interrupt Request)
 */

#include "MCP356xScale.h"

// Pin Definitions for SPI
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13

// Pin Definitions for ADCs
#define CS_PIN0 7
#define IRQ_PIN0 6
#define CS_PIN1 4
#define IRQ_PIN1 5
#define CS_PIN2 2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12
#define SAMPLE_DURATION 20 // Sample interval set to 20 ms

// Declaration of MCP356xScale object
MCP356xScale *mcpScale = nullptr;

// Global variables for timing and sampling
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned int sampleCount = 0;
float totalReading[TOTAL_NUM_CELLS] = {0};

/**
 * @brief Setup function run once at startup
 *
 * Initializes serial communication, the MCP356xScale instance,
 * and prepares for data collection.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Wait for the serial port to connect

  // Initialize the MCP356xScale object
  mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);

  // Perform tare for all the load cells
  mcpScale->tare(10000);

  startTime = millis(); // Initialize startTime to the current time
}

/**
 * @brief Main program loop
 *
 * Continuously reads from the ADCs, calculates average readings,
 * and sends the data via serial communication.
 */
void loop()
{
  // Continuously update ADC readings
  if (mcpScale->updatedAdcReadings())
  {
    sampleCount++;
    for (int i = 0; i < TOTAL_NUM_CELLS; i++)
    {
      totalReading[i] += mcpScale->getReading(i);
    }
  }

  // Check for serial commands
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("T,"))
    {
      int tareValue = command.substring(2).toInt(); // Extract number after "T,"
      if (tareValue > 0)
      {
        mcpScale->tare(tareValue); // Execute tare with specified iterations
      }
    }
  }

  // Check if it's time to calculate and send the average readings
  currentTime = millis();
  if (currentTime - startTime >= SAMPLE_DURATION)
  {
    int averageReading[TOTAL_NUM_CELLS];
    for (int i = 0; i < TOTAL_NUM_CELLS; i++)
    {
      averageReading[i] = round(totalReading[i] / sampleCount);
      totalReading[i] = 0; // Reset totalReading for the next sampling period
    }
    sampleCount = 0; // Reset sampleCount for the next sampling period

    // Construct and send the message
    char message[85];
    snprintf(message, 85, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
             averageReading[0], averageReading[1], averageReading[2],
             averageReading[3], averageReading[4], averageReading[5],
             averageReading[6], averageReading[7], averageReading[8],
             averageReading[9], averageReading[10], averageReading[11]);
    Serial.println(message);

    startTime = currentTime; // Update startTime for the next interval
  }
}