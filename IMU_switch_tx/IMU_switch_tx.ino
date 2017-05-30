//runs on Feather M0 RFM69HCW

#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "Terminal.h"
#include "Trace.h"

#include "calibration.h"
#include "RFaddress.h"

#define INTERRUPT_PIN 5
#define LED_PIN 13

#define TRANSMIT_INTERVAL 200 //ms

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//orig numbers we used on first model
#define ACC_MIN 100
#define ACC_MAX 2000
#define GYRO_MIN 300
#define GYRO_MAX 6000

//new numbers that we'll map to old numbers
#define INPUT_ACC_MIN 10
#define INPUT_ACC_MAX 10000
#define INPUT_GYRO_MIN 10
#define INPUT_GYRO_MAX 1000

#define VBATPIN A7

RH_RF69 driver(8, 3); //cs, irq
RHReliableDatagram manager(driver);

// Dont put this on the stack:
uint8_t RF_buf[RH_RF69_MAX_MESSAGE_LEN + 1];

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//
void setupRF()
{
	if (!manager.init())
		Serial.println("setupRF; init failed");
	
	if (!driver.setFrequency(915.0))
		Serial.println("setFrequency failed");

	driver.setTxPower(20);
	
	LoadAddresses(1, 0);
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

//
void sendBatteryVal()
{
	uint16_t val = analogRead(VBATPIN);
	float v = val * 2.0f * 3.3f / 1024.0f;
	tracef(F("sending battery val: %d (=%.1fV)\r\n"), val, v);
	
	uint8_t buf[4];
	buf[0] = 'b';
	buf[1] = val & 0xff;
	buf[2] = val >> 8;
	buf[3] = 0;
	sendData(addressData.dst, buf, sizeof(buf));
}

uint16_t accThreshold = ACC_MAX;
uint16_t gyroThreshold = GYRO_MAX;

uint8_t displayReadings = 0;

//
void setup()
{
	pinMode(LED_PIN, OUTPUT);
	
	setupRF();
	
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    if (mpu.testConnection())
    {
		Serial.println(F("MPU6050 connection successful"));
    }
    else
    {
		Serial.println(F("MPU6050 connection failed"));
		while(1); //spin
    }

	//wait for calibration if necessary
	LoadCalibration();
	if (calibrationData.signature != VALID_CALIBRATION_SIG)
	{
		Serial.println("calibration missing, waiting for commands");
		while (calibrationData.signature != VALID_CALIBRATION_SIG)
			processSerial();
	}

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

	ApplyCalibration();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
	{
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
	else
	{
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
		
		while(1); //spin
    }

	int lastTx = 0;
	uint16_t highestMagAcc = 0;
	uint16_t highestMagGyro = 0;
	
	//handshake
	while (1)
	{
		//send hello msg
		trace(F("sending hello\r\n"));
		sendData(addressData.dst, (uint8_t *)"h\x00", 2);
		
		//wait for reply
		uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
		uint8_t from;   
		if (manager.recvfromAckTimeout(RF_buf, &len, 2000, &from))
		{
			tracef(F("got reply from: %d:"), from);
			logRF(RF_buf, len);			
			
			//threshold settings
			if (RF_buf[0] == 't')
			{
				accThreshold = RF_buf[1] + (RF_buf[2] << 8);
				gyroThreshold = RF_buf[3] + (RF_buf[4] << 8);
				tracef(F("setting thresholds: acc:%d   gyro:%d\r\n"), accThreshold, gyroThreshold);
				break;
			}			
		}
		else
		{
			Serial.println(F("No reply, is destination running?"));
		}
	}		
		
	//main loop
	while (1)
	{
		processSerial();
		
		//message from receiver
		if (manager.available())
		{
			uint8_t len = RH_RF69_MAX_MESSAGE_LEN;
			uint8_t from;
			if (manager.recvfromAck(RF_buf, &len, &from))
			{
				tracef("got request from: %u\r\n", from);
				logRF(RF_buf, len);

				//threshold settings
				if (RF_buf[0] == 't')
				{
					accThreshold = RF_buf[1] + (RF_buf[2] << 8);
					gyroThreshold = RF_buf[3] + (RF_buf[4] << 8);
					tracef(F("setting thresholds. accThreshold:%d   gyroThreshold:%d\r\n"), accThreshold, gyroThreshold);
				}
				//battery query
				else if (RF_buf[0] == 'b')
				{
					sendBatteryVal();
				}
			}
		}
		
		// wait for MPU interrupt or extra packet(s) available
		if (!mpuInterrupt && fifoCount < packetSize)
			continue;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
			// reset so we can continue cleanly
			mpu.resetFIFO();
			Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else if (mpuIntStatus & 0x02)
		{
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;

			//grab real acceleration, adjusted to remove gravity
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGyro(&gg, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			
			uint16_t magAcc = (uint16_t)aaReal.getMagnitude();
			uint16_t magGyro = (uint16_t)gg.getMagnitude();
			
			//scale to original measurement system
			magAcc = map(magAcc, INPUT_ACC_MIN, INPUT_ACC_MAX, ACC_MIN, ACC_MAX);
			magGyro = map(magGyro, INPUT_GYRO_MIN, INPUT_GYRO_MAX, GYRO_MIN, GYRO_MAX);

			if (displayReadings)
				tracef(F("acc:%d thres:%d gyro:%d thres:%d\r\n"), magAcc, accThreshold, magGyro, gyroThreshold);

			//update highest during this period
			if (magAcc > highestMagAcc)
				highestMagAcc = magAcc;
			if (magGyro > highestMagGyro)
				highestMagGyro = magGyro;
			
			//send an update
			//if (millis() - lastTx >= TRANSMIT_INTERVAL) //time-based
			if (highestMagAcc >= accThreshold || highestMagGyro >= gyroThreshold) //threshold-based
			{
				uint8_t buf[10];
				buf[0] = 'd';
				buf[1] = accThreshold & 0xff;
				buf[2] = accThreshold >> 8;
				buf[3] = gyroThreshold & 0xff;
				buf[4] = gyroThreshold >> 8;
				buf[5] = highestMagAcc & 0xff;
				buf[6] = highestMagAcc >> 8;
				buf[7] = highestMagGyro & 0xff;
				buf[8] = highestMagGyro >> 8;
				buf[9] = 0;
				digitalWrite(LED_PIN, HIGH);
				sendData(addressData.dst, buf, sizeof(buf));
				digitalWrite(LED_PIN, LOW);
				lastTx = millis();
				highestMagAcc = 0.0f;
				highestMagGyro = 0.0f;
			}
		}
	}
}

//
void loop() {}
