//runs on Feather M0 RFM69HCW
//-this version is streamlined, no display or knobs, uses a selector switch instead

#include <Wire.h>
#include "Keyboard.h"
#include "Mouse.h"
#include <FlashStorage.h>

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#include "Trace.h"
#include "Terminal.h"

#include "thresholds.h"
#include "RFaddress.h"

#define RELAY_PIN 13
#define THRESHOLD_SELECT_LOW 10
#define THRESHOLD_SELECT_HIGH 11

#define RELAY_DURATION 500

FlashStorage(emulation_store, uint8_t);
uint8_t emulationAction = 0;

RH_RF69 driver(8, 3); //cs, irq
RHReliableDatagram manager(driver);

// Dont put this on the stack:
uint8_t RF_buf[RH_RF69_MAX_MESSAGE_LEN + 1];

//
void LoadEmulation()
{
	emulationAction = emulation_store.read();
}

//
void StoreEmulation()
{
	emulation_store.write(emulationAction);
}

//
void setupRF()
{
	if (!manager.init())
		Serial.println("setupRF; init failed");
	
	if (!driver.setFrequency(915.0))
		Serial.println("setFrequency failed");

	driver.setTxPower(20);

	LoadAddresses(0, 1);
	manager.setThisAddress(addressData.src);
	
	Serial.println("setupRF; initialized");
}

//
void logRF(uint8_t *buf, uint8_t len)
{
	for (int i = 0; i < len; i++)
		tracef("%u ", buf[i]);
	trace("\r\n");
}

//
int sendData(uint8_t addr, uint8_t *data, uint8_t txLen)
{
	//tracef("sending data to addr: %u\r\n", addr);
	//logRF(data, txLen);
	
	if (!manager.sendtoWait(data , txLen, addr))
	{
    	Serial.println(F("sendtoWait failed"));
    	return -1;
    }
}

void setup(void)
{
	Serial.begin(115200);
	trace(F("ready\r\n\r\n"));

	pinMode(RELAY_PIN, OUTPUT);
	
	LoadThresholds();
	LoadEmulation();
	
	for (int i = 0; i < 2; i++)
	{
		digitalWrite(RELAY_PIN, HIGH);
		delay(500);
		digitalWrite(RELAY_PIN, LOW);
		delay(500);
	}
	
	pinMode(THRESHOLD_SELECT_LOW, INPUT_PULLUP);
	pinMode(THRESHOLD_SELECT_HIGH, INPUT_PULLUP);
	
	setupRF();

	Mouse.begin();
	Keyboard.begin();
}

volatile uint16_t magAcc;
volatile uint16_t magGyro;

uint16_t accThreshold;
uint16_t gyroThreshold;

//
void sendThresholds()
{
	tracef(F("sendingThresholds. accThreshold:%d   gyroThreshold:%d\r\n"), accThreshold, gyroThreshold);
	
	uint8_t buf[6];
	buf[0] = 't';
	buf[1] = (uint16_t)accThreshold & 0xff;
	buf[2] = (uint16_t)accThreshold >> 8;
	buf[3] = (uint16_t)gyroThreshold & 0xff;
	buf[4] = (uint16_t)gyroThreshold >> 8;
	buf[5] = 0;
	sendData(addressData.dst, buf, sizeof(buf));
}

//
float getTxBattery()
{
	trace(F("sending tx battery query\r\n"));
	sendData(addressData.dst, (uint8_t *)"b\x00", 2);

	//wait for reply
	uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
	uint8_t from;   
	if (manager.recvfromAckTimeout(RF_buf, &len, 2000, &from))
	{
		tracef(F("got reply from: %d:"), from);
		logRF(RF_buf, len);			
		
		//battery value
		if (RF_buf[0] == 'b')
		{
			uint16_t val = RF_buf[1] + (RF_buf[2] << 8);
			float v = val * 2.0f * 3.3f / 1024.0f;
			return v;
		}			
	}
	
	return 0.0f;
}

uint8_t relayOn = 0;
uint32_t relayOnTime;

//
void onThresholdTriggered()
{
	trace(F("onThresholdTriggered\r\n"));
	
	//only fire emulation action at relay transition to on
	if (!relayOn)
	{
		switch(emulationAction)
		{
		case 0: break;
		case 1: Mouse.click(MOUSE_LEFT); break;
		case 2: Mouse.click(MOUSE_RIGHT); break;
		case 3: Keyboard.write(' '); break;
		case 4: Keyboard.write(KEY_RETURN); break;
		default: Keyboard.write(emulationAction); break;
		}
	}
	
	digitalWrite(RELAY_PIN, HIGH);
	relayOn = 1;
	
	relayOnTime = millis();
}

//
uint16_t prevAccThreshold;
uint16_t prevGyroThreshold;
void loop()
{
	processSerial();
	
	//turn off relay
	if (relayOn && millis() - relayOnTime > RELAY_DURATION)
	{
		digitalWrite(RELAY_PIN, LOW);
		relayOn = 0;
	}
	
	uint8_t thresholdID = 1;
	if (digitalRead(THRESHOLD_SELECT_LOW) == LOW)
		thresholdID = 2;
	else if (digitalRead(THRESHOLD_SELECT_HIGH) == LOW)
		thresholdID = 0;
	
	accThreshold = thresholdData.acc[thresholdID];
	gyroThreshold = thresholdData.gyro[thresholdID];

	if (accThreshold != prevAccThreshold || gyroThreshold != prevGyroThreshold)
	{
		sendThresholds();
		prevAccThreshold = accThreshold;
		prevGyroThreshold = gyroThreshold;
	}

	if (!manager.available())
		return;
	
	// Wait for a message addressed to us from the client
	uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
	uint8_t from;
	if (manager.recvfromAck(RF_buf, &len, &from))
	{
		tracef("got request from: %u\r\n", from);
		logRF(RF_buf, len);

		//sensor data
		if (RF_buf[0] == 'd')
		{
			uint16_t txAccThreshold = RF_buf[1] + (RF_buf[2] << 8);
			uint16_t txGyroThreshold = RF_buf[3] + (RF_buf[4] << 8);
			uint16_t magAcc = RF_buf[5] + (RF_buf[6] << 8);
			uint16_t magGyro = RF_buf[7] + (RF_buf[8] << 8);
			
			//mismatch, resend thresholds
			if (txAccThreshold != accThreshold || txGyroThreshold != gyroThreshold)
				sendThresholds();
			
			tracef(F("received: acc:%d thres:%d gyro:%d thres:%d\r\n"), magAcc, accThreshold, magGyro, gyroThreshold);
			if (magAcc > accThreshold || magGyro > gyroThreshold)
				onThresholdTriggered();
		}
		//hello msg
		else if (RF_buf[0] == 'h')
		{
			sendThresholds();
		}
	}
}
