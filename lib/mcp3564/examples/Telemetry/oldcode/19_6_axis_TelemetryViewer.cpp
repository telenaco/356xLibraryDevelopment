#include <MCP356x3axis.h>

 // Pin Definitions
#define SDI_PIN  11
#define SDO_PIN  12
#define SCK_PIN  13
#define CS_PIN0  7
#define IRQ_PIN0 6
#define CS_PIN1  4
#define IRQ_PIN1 5
#define CS_PIN2  2
#define IRQ_PIN2 3

#define TOTAL_NUM_CELLS 12

MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;

//uint32_t lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);

    // Tare operation for all load cells to set the current weight as the zero point
    for (int i = 0; i < TOTAL_NUM_CELLS; i++) { mcpScale->tare(i);}

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell2 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell3 = new MCP356x3axis(mcpScale, 6, 7, 8);
    loadCell4 = new MCP356x3axis(mcpScale, 9, 10, 11);

    BLA::Matrix<3, 3> calibMatrix1 = {
       -5.982118e-04,  -6.398772e-06, 1.493637e-05,
       -1.610696e-05,   5.980800e-04, 1.154983e-05,
        5.832123e-06,  -5.437381e-06, 6.200539e-04 };

    // For Load Cell 2
    BLA::Matrix<3, 3> calibMatrix2 = {
         5.981983e-04, -1.070736e-05, 7.760431e-06,
         2.589967e-06,  5.869149e-04, 6.217067e-06,
         5.954714e-06,  1.355531e-05, 5.982035e-04 };

    // For Load Cell 3
    BLA::Matrix<3, 3> calibMatrix3 = {
        -5.967505e-04, -1.410111e-05, -1.031143e-05,
        -1.276690e-05,  5.953475e-04, -4.933627e-06,
         8.278846e-06, -1.064470e-05, -5.746296e-04 };

    // For Load Cell 4
    BLA::Matrix<3, 3> calibMatrix4 = {
        -6.007485e-04, -1.092638e-05, -2.390023e-06,
        -1.328377e-05,  5.968730e-04,  5.299463e-06,
         3.958772e-06, -7.053156e-06, 6.141198e-04 };
    
    loadCell1->setCalibrationMatrix(calibMatrix1);
    loadCell2->setCalibrationMatrix(calibMatrix2);
    loadCell3->setCalibrationMatrix(calibMatrix3);
    loadCell4->setCalibrationMatrix(calibMatrix4);

    Serial.println("Load cells initialized and tared.");
}

void loop() {
    uint32_t startMicros = micros(); // Start time for this loop iteration

    // Update the ADC readings from all connected load cells
    if (mcpScale->updatedAdcReadings()) {

        StringBuilder output;
        
        // Construct the message with readings for each 3-axis load cell
        for (int i = 1; i <= 4; i++) {
            MCP356x3axis* currentLoadCell = nullptr;
            switch(i) {
                case 1: currentLoadCell = loadCell1; break;
                case 2: currentLoadCell = loadCell2; break;
                case 3: currentLoadCell = loadCell3; break;
                case 4: currentLoadCell = loadCell4; break;
            }

            Matrix<3, 1> gfReading = currentLoadCell->getGfReading();

            // Append the integer values of grams-force for each axis to the output
            output.concat((int)round(gfReading(0)));
            output.concat(',');
            output.concat((int)round(gfReading(1)));
            output.concat(',');
            output.concat((int)round(gfReading(2)));
            if (i < 4) {
                output.concat(','); // Separator between load cell readings
            }
        }

        // Append the elapsed time in microseconds to the output
        output.concat(',');
        int ellapsedTime = micros() - startMicros;
        output.concat(ellapsedTime);

        // Send the combined grams-force readings and elapsed time for all load cells over serial
        Serial.println((char*)output.string());
    }
}
