#ifndef __THRESHOLDS_H_
#define __THRESHOLDS_H_

#define VALID_THRESHOLDS_SIG 0xDEADC0DE

//
struct ThresholdData
{
	//low, medium, high
	uint16_t acc[3];
	uint16_t gyro[3];
    uint32_t signature;
};

extern ThresholdData thresholdData;

void LoadThresholds();
void DumpThresholds();
void StoreThresholds();

#endif
