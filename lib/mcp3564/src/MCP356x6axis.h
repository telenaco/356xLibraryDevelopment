#ifndef SIX_AXIS_LOAD_CELL_H
#define SIX_AXIS_LOAD_CELL_H

#include "MCP356x3axis.h"

class MCP356x6axis {
public:
    MCP356x6axis(MCP356x3axis* loadCellA, MCP356x3axis* loadCellB, MCP356x3axis* loadCellC, MCP356x3axis* loadCellD, float plateWidth, float plateLength);
    ~MCP356x6axis();

    void setCalibrationMatrix(const BLA::Matrix<6, 6>& calibMatrix);
    BLA::Matrix<6, 6> getCalibrationMatrix();

    BLA::Matrix<6, 1> readRawForceAndTorque();
    BLA::Matrix<6, 1> readCalibratedForceAndTorque();

    void tare();
    void reset();

    // Add more functions as needed

private:
    MCP356x3axis* _loadCellA;
    MCP356x3axis* _loadCellB;
    MCP356x3axis* _loadCellC;
    MCP356x3axis* _loadCellD;

    float _plateWidth;
    float _plateLength;

    BLA::Matrix<6, 6> _calibrationMatrix;

};

#endif // SIX_AXIS_LOAD_CELL_H