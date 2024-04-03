#include <MCP356x6axis.h>

// Pin Definitions
#define SDI_PIN 11
#define SDO_PIN 12
#define SCK_PIN 13
#define CS_PIN0 7
#define IRQ_PIN0 6
#define CS_PIN1 4
#define IRQ_PIN1 5
#define CS_PIN2 2
#define IRQ_PIN2 3
#define MOTOR_PWM_PIN 9 // PWM pin connected to the motor controller
#define RAMP_UP_DURATION 10000 // 10 seconds ramp up duration
#define RAMP_DOWN_DURATION 10000 // 10 seconds ramp down duration
#define HOLD_DURATION 5000 // 5 seconds hold duration at max and min
#define TOTAL_CYCLE_DURATION (RAMP_UP_DURATION + HOLD_DURATION + RAMP_DOWN_DURATION + HOLD_DURATION)

#define TOTAL_NUM_CELLS 12
#define SAMPLE_INTERVAL 10 // Sample interval set to 20ms

MCP356xScale* mcpScale = nullptr;
MCP356x3axis* loadCell1 = nullptr;
MCP356x3axis* loadCell2 = nullptr;
MCP356x3axis* loadCell3 = nullptr;
MCP356x3axis* loadCell4 = nullptr;
MCP356x6axis* sixAxisLoadCell = nullptr;

unsigned long cycleStartTime;
bool cycleComplete = false;
int lastSentPWMValue = -1;

// Variables for averaging
unsigned int sampleCount = 0;
Matrix<3, 1> totalReadings1 = { 0, 0, 0 };
Matrix<3, 1> totalReadings2 = { 0, 0, 0 };
Matrix<3, 1> totalReadings3 = { 0, 0, 0 };
Matrix<3, 1> totalReadings4 = { 0, 0, 0 };

void initializeCalibrationWithMatrices() {
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
}

void initializeCalibrationWithpolynomial() {
    // Set Polynomial Calibration for Load Cell 1
    loadCell1->setCalibrationPolynomial(
        { -2.828916e+02, -6.139373e-04, -4.779867e-12 }, // X Axis
        { 3.592542e+02, 5.951776e-04, 1.063187e-11 },    // Y Axis
        { -5.722137e+02, 6.244786e-04, -9.493906e-12 }   // Z Axis
    );

    // Set Polynomial Calibration for Load Cell 2
    loadCell2->setCalibrationPolynomial(
        { -1.517346e+02, 6.002052e-04, -7.213967e-13 }, // X Axis
        { 3.217731e+02, 5.941050e-04, -7.526178e-12 },  // Y Axis
        { -6.883740e+02, 5.964100e-04, 4.769434e-12 }   // Z Axis
    );

    loadCell3->setCalibrationPolynomial(
        { 4.924497e+02, -6.000414e-04, -5.842038e-12 }, // X Axis
        { 4.065573e+02, 5.873697e-04, -3.809444e-12 },  // Y Axis
        { -3.722284e+02, -5.611561e-04, 4.556780e-12 }  // Z Axis
    );

    loadCell4->setCalibrationPolynomial(
        { -1.973620e+01, -6.040589e-04, -8.463602e-13 }, // X Axis
        { 5.351286e+02, 5.982422e-04, 4.519417e-12 },    // Y Axis
        { -2.636953e+02, 5.996329e-04, 5.869018e-12 }    // Z Axis
    );
    loadCell1->tare(100);
    loadCell2->tare(100);
    loadCell3->tare(100);
    loadCell4->tare(100);
}

int rampController() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - cycleStartTime;
    unsigned long phaseTime = elapsedTime % TOTAL_CYCLE_DURATION;
    int pwmValue = 0;

    // Determine the current PWM value based on the ramp-up/down and hold phases
    if (phaseTime <= RAMP_UP_DURATION) {
        pwmValue = map(phaseTime, 0, RAMP_UP_DURATION, 0, 255);
    }
    else if (phaseTime <= RAMP_UP_DURATION + HOLD_DURATION) {
        pwmValue = 255;
    }
    else if (phaseTime <= RAMP_UP_DURATION + HOLD_DURATION + RAMP_DOWN_DURATION) {
        pwmValue = map(phaseTime, RAMP_UP_DURATION + HOLD_DURATION, RAMP_UP_DURATION + HOLD_DURATION + RAMP_DOWN_DURATION, 255, 0);
    }
    else {
        pwmValue = 0;
    }

    // when the PWM changes update the actuator 
    if (pwmValue != lastSentPWMValue) {
        analogWrite(MOTOR_PWM_PIN, pwmValue);   // controlling the actuator via PWM
        Serial1.println(pwmValue);              // controlling the actuator via Serial message
        // If using an ESC, replace analogWrite with ESC-specific command
        // esc.setSpeed(map(newPWMValue, 0, 255, ESC_MIN_VALUE, ESC_MAX_VALUE));
        lastSentPWMValue = pwmValue; // Update the last sent value
    }

    // Restart the cycle automatically for continuous operation
    if (elapsedTime > TOTAL_CYCLE_DURATION) {
        cycleStartTime = currentTime; // Reset cycle start time
    }

    return pwmValue; // Optional, depending on if you need the value in loop()
}


void setup() {
    // Serial to communicate with the computer
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // if you want to interact with your haptic device controller via serial 
    Serial1.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize the scale for managing multiple load cells with specified pins
    mcpScale = new MCP356xScale(TOTAL_NUM_CELLS, SCK_PIN, SDO_PIN, SDI_PIN, IRQ_PIN0, CS_PIN0, IRQ_PIN1, CS_PIN1, IRQ_PIN2, CS_PIN2);
    mcpScale->tare(100000);

    // Initialize the 3-axis load cell with indices corresponding to each axis
    loadCell1 = new MCP356x3axis(mcpScale, 9, 10, 11);
    loadCell2 = new MCP356x3axis(mcpScale, 0, 1, 2);
    loadCell3 = new MCP356x3axis(mcpScale, 3, 4, 5);
    loadCell4 = new MCP356x3axis(mcpScale, 6, 7, 8);

    //initializeCalibrationWithpolynomial();
    initializeCalibrationWithMatrices();

    loadCell1->setAxisInversion(1, 1, 1);
    loadCell2->setAxisInversion(1, 0, 1);
    loadCell3->setAxisInversion(1, 1, 0);
    loadCell4->setAxisInversion(1, 1, 0);

    // Initialize the 6-axis load cell using the four 3-axis load cells
    float plateWidth = 0.125; // Example plate width in mm
    float plateLength = 0.125; // Example plate length in mm
    sixAxisLoadCell = new MCP356x6axis(loadCell1, loadCell2, loadCell3, loadCell4, plateWidth, plateLength);

    // Define the calibration matrix as provided
    Matrix<6, 6> calibMatrix = {
   0.99962,   0.010247,   0.0125986,  -0.000360691, -0.0217137,   0.00507714,
  -0.0302539, 0.994631,   0.0150176,   0.025775,     0.000985963, 0.00494589,
  -0.0090635, -0.0158627, 0.994607,    0.00113301,  -0.00941599,  -0.0000305863,
  -0.113791,  -0.111746,  -0.116821,   1.14718,      0.0195911,   -0.0137317,
  -0.401589,  -0.0569284, 0.588532,    0.0120469,   1.15636,      0.0465735,
  -0.0173116,  0.00683699, 0.106079,   0.0170052,   -0.00432406,  1.00594 };

    // Apply the calibration matrix
    sixAxisLoadCell->setCalibrationMatrix(calibMatrix);

    pinMode(MOTOR_PWM_PIN, OUTPUT);
    analogWrite(MOTOR_PWM_PIN, 0); // Start with motor off

    cycleStartTime = millis();
}

void loop() {

    int currentPWMValue = rampController(); // Adjust controller actuator for ramp-up, hold, ramp-down, and hold, repeat

    if (mcpScale->updatedAdcReadings()) {
        BLA::Matrix<6, 1> forceAndTorque = sixAxisLoadCell->readCalibratedForceAndTorque();

        // Prepare and send the data string including microsecond timestamp and PWM value
        StringBuilder output;
        output.concatf("%lu,", micros()); // Add microsecond timestamp
        for (int i = 0; i < 6; i++) {
            output.concatf("%.3f,", forceAndTorque(i)); // Add force and torque readings
        }
        output.concatf("%d", currentPWMValue); // Add current PWM value

        Serial.println((char*)output.string());
    }
}