/**
 * @file MCP356x6axis.cpp
 * @brief Implementation of the MCP356x6axis class for 6-axis force/torque sensing.
 * 
 * This file contains the implementation of the MCP356x6axis class, which integrates
 * four 3-axis load cells to create a 6-axis force/torque sensor system.
 * 
 * @author Jose Luis Berna Moya
 * @date March 2025
 */

 #include "MCP356x6axis.h"

 /**
  * @brief Constructor for the MCP356x6axis class.
  * 
  * Initializes the 6-axis force/torque sensor with four 3-axis load cells and
  * sets up the initial calibration matrix as an identity matrix.
  * 
  * @param loadCellA Pointer to the first 3-axis load cell
  * @param loadCellB Pointer to the second 3-axis load cell
  * @param loadCellC Pointer to the third 3-axis load cell
  * @param loadCellD Pointer to the fourth 3-axis load cell
  * @param plateWidth Width of the mounting plate in meters
  * @param plateLength Length of the mounting plate in meters
  */
 MCP356x6axis::MCP356x6axis(MCP356x3axis* loadCellA, MCP356x3axis* loadCellB, 
                            MCP356x3axis* loadCellC, MCP356x3axis* loadCellD, 
                            float plateWidth, float plateLength)
     : _loadCellA(loadCellA), 
       _loadCellB(loadCellB), 
       _loadCellC(loadCellC), 
       _loadCellD(loadCellD), 
       _plateWidth(plateWidth), 
       _plateLength(plateLength) {
     
     // Initialize the calibration matrix to an identity matrix
     _calibrationMatrix = Identity<6, 6>();
 }
 
 /**
  * @brief Destructor for the MCP356x6axis class.
  * 
  * No dynamic memory allocation is performed by this class, so no cleanup is needed.
  */
 MCP356x6axis::~MCP356x6axis() {
     // No dynamic memory allocation, so no cleanup needed
 }
 
 /**
  * @brief Sets the calibration matrix for the 6-axis sensor.
  * 
  * @param calibMatrix The 6x6 calibration matrix to be applied
  */
 void MCP356x6axis::setCalibrationMatrix(const BLA::Matrix<6, 6>& calibMatrix) {
     _calibrationMatrix = calibMatrix;
 }
 
 /**
  * @brief Gets the current calibration matrix.
  * 
  * @return BLA::Matrix<6, 6> The current calibration matrix
  */
 BLA::Matrix<6, 6> MCP356x6axis::getCalibrationMatrix() {
     return _calibrationMatrix;
 }
 
 /**
  * @brief Reads raw force and torque values from the load cells.
  * 
  * This method reads the forces from all four 3-axis load cells and computes
  * the combined forces and torques for all six degrees of freedom.
  * 
  * @return BLA::Matrix<6, 1> Vector of raw force and torque values [Fx, Fy, Fz, Mx, My, Mz]
  */
 BLA::Matrix<6, 1> MCP356x6axis::readRawForceAndTorque() {
     BLA::Matrix<6, 1> rawForceAndTorque;
 
     // Read the force values from each load cell in Newtons
     BLA::Matrix<3, 1> forceA = _loadCellA->getNewtonReading();
     BLA::Matrix<3, 1> forceB = _loadCellB->getNewtonReading();
     BLA::Matrix<3, 1> forceC = _loadCellC->getNewtonReading();
     BLA::Matrix<3, 1> forceD = _loadCellD->getNewtonReading();
 
     // Calculate the net forces (sum of individual forces)
     rawForceAndTorque(0) = forceA(0) + forceB(0) + forceC(0) + forceD(0);  // Fx
     rawForceAndTorque(1) = forceA(1) + forceB(1) + forceC(1) + forceD(1);  // Fy
     rawForceAndTorque(2) = forceA(2) + forceB(2) + forceC(2) + forceD(2);  // Fz
     
     // Calculate the torques (moments) based on force distribution and geometry
     // Mx = torque around X axis = (plateWidth/2) * Fz distribution
     rawForceAndTorque(3) = (_plateWidth / 2.0f) * 
                           (-forceA(2) - forceB(2) + forceC(2) + forceD(2));
     
     // My = torque around Y axis = (plateLength/2) * Fz distribution
     rawForceAndTorque(4) = (_plateLength / 2.0f) * 
                           (-forceA(2) + forceB(2) + forceC(2) - forceD(2));
     
     // Mz = torque around Z axis = contributions from Fx and Fy distributions
     rawForceAndTorque(5) = (_plateLength / 2.0f) * 
                           (forceA(1) - forceB(1) - forceC(1) + forceD(1)) +
                            (_plateWidth / 2.0f) * 
                           (forceA(0) + forceB(0) - forceC(0) - forceD(0));
 
     return rawForceAndTorque;
 }
 
 /**
  * @brief Reads calibrated force and torque values from the load cells.
  * 
  * Applies the calibration matrix to the raw force and torque readings
  * to obtain calibrated values for all six axes.
  * 
  * @return BLA::Matrix<6, 1> Vector of calibrated force and torque values
  */
 BLA::Matrix<6, 1> MCP356x6axis::readCalibratedForceAndTorque() {
     BLA::Matrix<6, 1> rawForceAndTorque = readRawForceAndTorque();
     return _calibrationMatrix * rawForceAndTorque;
 }
 
 /**
  * @brief Performs a tare operation on all load cells.
  * 
  * This method zero-calibrates all four load cells to establish a baseline.
  * Currently commented out for development purposes.
  */
 void MCP356x6axis::tare() {
     // Implementation for taring all four load cells
     _loadCellA->tare();
     _loadCellB->tare();
     _loadCellC->tare();
     _loadCellD->tare();
 }
 
 /**
  * @brief Resets the calibration for all load cells and the 6-axis sensor.
  * 
  * This method resets all calibration matrices to identity matrices,
  * effectively removing any calibration adjustments.
  */
 void MCP356x6axis::reset() {
     // Reset the individual load cell calibration matrices to identity
     _loadCellA->setCalibrationMatrix(Identity<3, 3>());
     _loadCellB->setCalibrationMatrix(Identity<3, 3>());
     _loadCellC->setCalibrationMatrix(Identity<3, 3>());
     _loadCellD->setCalibrationMatrix(Identity<3, 3>());
     
     // Reset the 6-axis calibration matrix to identity
     _calibrationMatrix = Identity<6, 6>();
 }