#include <Arduino.h>
#include <stdlib.h>
#include <RHReliableDatagram.h>

#include "Trace.h"

#include "calibration.h"
#include "RFaddress.h"

#define IMU_SWITCH_TX_BUILD_VERSION "1"

#define VBATPIN A7

#define CMD_BUF_SIZE 64

extern RHReliableDatagram manager;
extern uint16_t accThreshold;
extern uint16_t gyroThreshold;
extern uint8_t displayReadings;

//
void terminalCmdHandler(char *cmd)
{
	if (strcmp(cmd, "help") == 0)
	{
		tracef("TRANSMITTER version:%s\r\n", IMU_SWITCH_TX_BUILD_VERSION);
		trace("command list:\r\n");
		trace("tx_bat\r\n");
		trace("dump_thresholds\r\n");
		trace("set_thresholds <acc> <gyro>\r\n");
		trace("dump_calibration\r\n");
		trace("set_calibration <accX> <accY> <accZ> <gyroX> <gyroY> <gyroZ>\r\n");
		trace("apply_calibration\r\n");
		trace("run_calibration\r\n");
		trace("display <readings enabled: 1/0>\r\n");
		trace("dump_addresses\r\n");
		trace("set_addresses <src> <dst>\r\n");
	}
	else if (strcmp(cmd, "tx_bat") == 0)
	{
		float v = analogRead(VBATPIN) * 2.0f * 3.3f / 1024.0f;
		tracef("tx battery: %.1fV\r\n", v);		
	}
	else if (strcmp(cmd, "dump_thresholds") == 0)
	{
		tracef(F("thresholds: acc:%d   gyro:%d\r\n"), accThreshold, gyroThreshold);
	}
	else if (strcmp(cmd, "set_thresholds") == 0)
	{
		accThreshold = atoi(strtok(NULL, " \r\n"));
		gyroThreshold = atoi(strtok(NULL, " \r\n"));
		tracef(F("thresholds: acc:%d   gyro:%d\r\n"), accThreshold, gyroThreshold);
	}
	else if (strcmp(cmd, "dump_calibration") == 0)
	{
		DumpCalibration();
	}
	else if (strcmp(cmd, "set_calibration") == 0)
	{
		calibrationData.accelX = atoi(strtok(NULL, " \r\n"));
		calibrationData.accelY = atoi(strtok(NULL, " \r\n"));
		calibrationData.accelZ = atoi(strtok(NULL, " \r\n"));
		calibrationData.gyroX = atoi(strtok(NULL, " \r\n"));
		calibrationData.gyroY = atoi(strtok(NULL, " \r\n"));
		calibrationData.gyroZ = atoi(strtok(NULL, " \r\n"));
		calibrationData.signature = VALID_CALIBRATION_SIG;
		StoreCalibration();
	}
	else if (strcmp(cmd, "apply_calibration") == 0)
	{
		ApplyCalibration();
	}
	else if (strcmp(cmd, "run_calibration") == 0)
	{
		RunCalibration();
	}
	else if (strcmp(cmd, "display") == 0)
	{
		displayReadings = atoi(strtok(NULL, " \r\n"));
	}
	else if (strcmp(cmd, "dump_addresses") == 0)
	{
		DumpAddresses();
	}
	else if (strcmp(cmd, "set_addresses") == 0)
	{
		addressData.src = atoi(strtok(NULL, " \r\n"));
		addressData.dst = atoi(strtok(NULL, " \r\n"));
		addressData.signature = VALID_ADDRESS_SIG;
		StoreAddresses();
		manager.setThisAddress(addressData.src);
	}
	else if (cmd)
	{
		trace("ERROR\r\n");
	}
}
