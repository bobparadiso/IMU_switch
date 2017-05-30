#include <FlashStorage.h>
#include "Trace.h"
#include "thresholds.h"

FlashStorage(threshold_store, ThresholdData);
ThresholdData thresholdData;

//
void DumpThresholds()
{
	tracef("Threshold Data\r\n\tLO: acc:%d   gyro:%d\r\n\tMED: acc:%d   gyro:%d\r\n\tHI: acc:%d   gyro:%d\r\n",
		thresholdData.acc[0], thresholdData.gyro[0],
		thresholdData.acc[1], thresholdData.gyro[1],
		thresholdData.acc[2], thresholdData.gyro[2]);
}

//
void StoreThresholds()
{
	Serial.println("storing thresholds:");
	DumpThresholds();

    threshold_store.write(thresholdData);	
}

//
void ApplyDefaultThresholds()
{
	thresholdData.acc[0] = 275;
	thresholdData.gyro[0] = 700;
	
	thresholdData.acc[1] = 350;
	thresholdData.gyro[1] = 900;
	
	thresholdData.acc[2] = 450;
	thresholdData.gyro[2] = 1100;
	
	thresholdData.signature = VALID_THRESHOLDS_SIG;
	StoreThresholds();
}

//
void LoadThresholds()
{
	thresholdData = threshold_store.read();	
	if (thresholdData.signature != VALID_THRESHOLDS_SIG)
		ApplyDefaultThresholds();
	
	DumpThresholds();
}
