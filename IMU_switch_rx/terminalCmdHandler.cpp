#include <Arduino.h>
#include <stdlib.h>
#include <RHReliableDatagram.h>

#include "Trace.h"

#include "thresholds.h"
#include "RFaddress.h"

#define IMU_SWITCH_RX_BUILD_VERSION "1"

#define VBATPIN A7

extern RHReliableDatagram manager;
extern uint8_t emulationAction;

float getTxBattery();
void onThresholdTriggered();
void StoreEmulation();
 
//
void terminalCmdHandler(char *cmd)
{
	if (strcmp(cmd, "help") == 0)
	{
		tracef("RECEIVER version:%s\r\n", IMU_SWITCH_RX_BUILD_VERSION);
		trace("command list:\r\n");
		trace("rx_bat\r\n");
		trace("tx_bat\r\n");
		trace("dump_thresholds\r\n");
		trace("set_thresholds <acc_lo> <gyro_lo> <acc_med> <gyro_med> <acc_hi> <gyro_hi>\r\n");
		trace("dump_addresses\r\n");
		trace("set_addresses <src> <dst>\r\n");
		trace("dump_emulation\r\n");
		trace("set_emulation <mode: 0=NONE, 1=LEFT_CLICK, 2=RIGHT_CLICK, 3=SPACE, 4=ENTER, else ASCII_CODE>\r\n");
		trace("test_emulation <delay in ms>\r\n");
	}
	else if (strcmp(cmd, "rx_bat") == 0)
	{
		float v = analogRead(VBATPIN) * 2.0f * 3.3f / 1024.0f;
		tracef("rx battery: %.1fV\r\n", v);		
	}
	else if (strcmp(cmd, "tx_bat") == 0)
	{
		float v = getTxBattery();
		tracef("tx battery: %.1fV\r\n", v);		
	}
	else if (strcmp(cmd, "dump_thresholds") == 0)
	{
		DumpThresholds();
	}
	else if (strcmp(cmd, "set_thresholds") == 0)
	{
		thresholdData.acc[0] = atoi(strtok(NULL, " \r\n"));
		thresholdData.gyro[0] = atoi(strtok(NULL, " \r\n"));
		thresholdData.acc[1] = atoi(strtok(NULL, " \r\n"));
		thresholdData.gyro[1] = atoi(strtok(NULL, " \r\n"));
		thresholdData.acc[2] = atoi(strtok(NULL, " \r\n"));
		thresholdData.gyro[2] = atoi(strtok(NULL, " \r\n"));
		StoreThresholds();
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
	else if (strcmp(cmd, "dump_emulation") == 0)
	{
		tracef("emulation: %d\r\n", emulationAction);
	}
	else if (strcmp(cmd, "set_emulation") == 0)
	{
		emulationAction = atoi(strtok(NULL, " \r\n"));
		StoreEmulation();
		tracef("emulation: %d\r\n", emulationAction);
	}
	else if (strcmp(cmd, "test_emulation") == 0)
	{
		uint32_t delayTime = atoi(strtok(NULL, " \r\n"));
		delay(delayTime);
		onThresholdTriggered();
	}
	else if (cmd)
	{
		trace("ERROR\r\n");
	}
}
