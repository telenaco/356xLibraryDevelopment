// #include "MCP356x3axis.h"

// // Constructor implementation
// MCP356x3axis::MCP356x3axis(MCP356xScale *xScale, MCP356xScale *yScale, MCP356xScale *zScale)
//     : xScale(xScale), yScale(yScale), zScale(zScale) {
// }

// // Destructor implementation
// MCP356x3axis::~MCP356x3axis() {
//     // Cleanup code (if any)
// }

// // digitalReading3D method implementation
// Reading3D MCP356x3axis::digitalReading3D() {
//     Reading3D digitalReadings;

//     // Retrieve digital force readings from each scale
//     digitalReadings.x = xScale->getDigitalValue();
//     digitalReadings.y = yScale->getDigitalValue();
//     digitalReadings.z = zScale->getDigitalValue();

//     return digitalReadings;
// }

// Force3D MCP356x3axis::ForceData3D() {
//     Force3D forceReadings;
//     forceReadings.x = xScale->convertToPolynomialForce();
//     forceReadings.y = yScale->convertToPolynomialForce();
//     forceReadings.z = zScale->convertToPolynomialForce();

//     return forceReadings;
// }

// void MCP356x3axis::setCalibrationCoefficients(const float coeffsX[3], const float coeffsY[3], const float coeffsZ[3]) {
//     xScale->setPolynomialCalibration(coeffsX[2], coeffsX[1], coeffsX[0]);
//     yScale->setPolynomialCalibration(coeffsY[2], coeffsY[1], coeffsY[0]);
//     zScale->setPolynomialCalibration(coeffsZ[2], coeffsZ[1], coeffsZ[0]);
//     _isCalibrated = true;
// }

// void MCP356x3axis::tareScales() {
//     Serial.println("Taring X");
//     xScale->tare();
//     Serial.println("Taring Y");
//     yScale->tare();
//     Serial.println("Taring Z");
//     zScale->tare();
// }

