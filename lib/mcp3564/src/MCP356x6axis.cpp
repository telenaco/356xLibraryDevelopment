#include "MCP356x6axis.h"

MCP356x6axis::MCP356x6axis(MCP356x3axis* loadCellA, MCP356x3axis* loadCellB, MCP356x3axis* loadCellC, MCP356x3axis* loadCellD, float plateWidth, float plateLength)
    : _loadCellA(loadCellA), _loadCellB(loadCellB), _loadCellC(loadCellC), _loadCellD(loadCellD), _plateWidth(plateWidth), _plateLength(plateLength) {
    // Initialize the calibration matrix to an identity matrix
    _calibrationMatrix = Identity<6, 6>();
}

MCP356x6axis::~MCP356x6axis() {
    // No dynamic memory allocation, so no cleanup needed
}

void MCP356x6axis::setCalibrationMatrix(const BLA::Matrix<6, 6>& calibMatrix) {
    _calibrationMatrix = calibMatrix;
}

BLA::Matrix<6, 6> MCP356x6axis::getCalibrationMatrix() {
    return _calibrationMatrix;
}

BLA::Matrix<6, 1> MCP356x6axis::readRawForceAndTorque() {
    BLA::Matrix<6, 1> rawForceAndTorque;

    // Read the force values from each load cell
    BLA::Matrix<3, 1> forceA = _loadCellA->getNewtonReading();
    BLA::Matrix<3, 1> forceB = _loadCellB->getNewtonReading();
    BLA::Matrix<3, 1> forceC = _loadCellC->getNewtonReading();
    BLA::Matrix<3, 1> forceD = _loadCellD->getNewtonReading();

    // Calculate the net forces and moments using Equation 6
    rawForceAndTorque(0) = forceA(0) + forceB(0) + forceC(0) + forceD(0);  // Fx
    rawForceAndTorque(1) = forceA(1) + forceB(1) + forceC(1) + forceD(1);  // Fy
    rawForceAndTorque(2) = forceA(2) + forceB(2) + forceC(2) + forceD(2);  // Fz
    rawForceAndTorque(3) = (_plateWidth / 2.0f) * (-forceA(2) - forceB(2) + forceC(2) + forceD(2));  // Mx
    rawForceAndTorque(4) = (_plateLength / 2.0f) * (-forceA(2) + forceB(2) + forceC(2) - forceD(2));  // My
    rawForceAndTorque(5) = (_plateLength / 2.0f) * (forceA(1) - forceB(1) - forceC(1) + forceD(1)) +
                           (_plateWidth / 2.0f) * (forceA(0) + forceB(0) - forceC(0) - forceD(0));  // Mz

    return rawForceAndTorque;
}

BLA::Matrix<6, 1> MCP356x6axis::readCalibratedForceAndTorque() {
    BLA::Matrix<6, 1> rawForceAndTorque = readRawForceAndTorque();
    return _calibrationMatrix * rawForceAndTorque;
}

void MCP356x6axis::tare() {
    // _loadCellA->tare();
    // _loadCellB->tare();
    // _loadCellC->tare();
    // _loadCellD->tare();
}

void MCP356x6axis::reset() {
    // Reset the load cells and clear the calibration matrix
    _loadCellA->setCalibrationMatrix(Identity<3, 3>());
    _loadCellB->setCalibrationMatrix(Identity<3, 3>());
    _loadCellC->setCalibrationMatrix(Identity<3, 3>());
    _loadCellD->setCalibrationMatrix(Identity<3, 3>());
    _calibrationMatrix = Identity<6, 6>();
}