#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

#define VALID_CALIBRATION_SIG 0xDEADC0DE

//
struct CalibrationData
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    uint32_t signature;
};

extern CalibrationData calibrationData;

void LoadCalibration();
void StoreCalibration();
void ApplyCalibration();
void RunCalibration();
void DumpCalibration();

#endif
