/**
  ******************************************************************************
  * @file	uArmController.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONTROLLER_H_
#define _UARMCONTROLLER_H_

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "UFServo.h"
#include "uArmConfig.h"
#include "uArmPin.h"
#include "uArmTypes.h"




#define DEFAULT_ANGLE			60

#define MATH_PI 			3.141592653589793238463
#define MATH_TRANS  		57.2958    





#ifdef MKII
	#define MATH_L1 			88.9	//90.00	
	#define MATH_L2 			10		//21.17	
	#define MATH_LOWER_ARM 		142.07	//148.25	
	#define MATH_UPPER_ARM 		158.8	//160.2 	
	#define MATH_FRONT_HEADER 	29.4	//25.00// the distance between wrist to the front point we use
	#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM
	#define MAX_Z				260		// MAX height
	#define MIN_Z				(-120)
#elif defined(METAL)
	#define MATH_L1 			107.45	
	#define MATH_L2 			21.17	
	#define MATH_LOWER_ARM 		148.25	
	#define MATH_UPPER_ARM 		160.2 	
	#define MATH_FRONT_HEADER 	25.00// the distance between wrist to the front point we use
	#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM
	#define MAX_Z				260		// max height
	#define MIN_Z				(-120)
#elif defined(SWIFT)
	#define MATH_L1 			88.9	//90.00	
	#define MATH_L2 			10		//21.17	
	#define MATH_LOWER_ARM 		142.07	//148.25	
	#define MATH_UPPER_ARM 		158.8	//160.2 	
	#define MATH_FRONT_HEADER 	29.4	//25.00// the distance between wrist to the front point we use
	#define MATH_UPPER_LOWER 	MATH_UPPER_ARM/MATH_LOWER_ARM
	#define MAX_Z				260		// MAX height
	#define MIN_Z				(-120)	

	#define HW_PIN_0			33
	#define HW_PIN_1			32
	#define HW_PIN_2			31
	#define HW_PIN_3			30
	
#endif




#define LOWER_ARM_MAX_ANGLE      130
#define LOWER_ARM_MIN_ANGLE      0
#define UPPER_ARM_MAX_ANGLE      120
#define UPPER_ARM_MIN_ANGLE      0
#define LOWER_UPPER_MAX_ANGLE    151
#define LOWER_UPPER_MIN_ANGLE    10

#define LINEAR_INTERCEPT_START_ADDRESS      70
#define LINEAR_SLOPE_START_ADDRESS          50
#define MANUAL_OFFSET_ADDRESS               30
#define OFFSET_STRETCH_START_ADDRESS        20
#define SERIAL_NUMBER_ADDRESS               100

#define LINEAR_CALIBRATION_FLAG_ADDREESS	200
#define ADC_CALIBRATION_FLAG_ADDRESS		210
#define OFFSET_CALIBRATION_FLAG_ADDRESS		220
#define MAC_FLAG_ADDRESS					240
#define MAC_ADDRESS					242

#define SERVO_9G_MAX    460
#define SERVO_9G_MIN    98

#define SERVO_MAX    630
#define SERVO_MIN    70

#define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2

#define DATA_LENGTH  0x40

#define LEFT_SERVO_ADDRESS   (2048+0x0000)
#define RIGHT_SERVO_ADDRESS  (2048+0x02D0)
#define ROT_SERVO_ADDRESS    (2048+0x05A0)

#define ADC_START_ADDRESS	4096

#define EEPROM_WRITE_TEST_ADDR	1024

#define MAX_MANUAL_OFFSET	15

#define ADC_FALG_DATA_INDEX	100



class uArmController
{
public:
	uArmController();

	void init();

	void attachAllServo();
	void attachServo(byte servoNum);
	void detachServo(byte servoNum);
	void detachAllServo();

	double getReverseServoAngle(byte servoNum, double servoAngle);
	void writeServoAngle(double servoRotAngle, double servoLeftAngle, double servoRightAngle);
	void writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset = true);
	double readServoAngle(byte servoNum, boolean withOffset = true);
	double readServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle, boolean withOffset = true);	
	void updateAllServoAngle(boolean withOffset = true);

	double getServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle);
	double getServeAngle(byte servoNum);
	unsigned char validateAngle(byte srevoNum, double &angle);

	#if defined(MKII) || defined(SWIFT)
	void readServoCalibrationData(unsigned int address, double& angle);	
	#endif

	unsigned char getCurrentXYZ(double& x, double& y, double& z);
	//unsigned char getXYZFromPolar(double& x, double& y, double& z, double s, double r, double h);
	unsigned char getXYZFromAngle(double& x, double& y, double& z, double rot, double left, double right);

	unsigned char setServoSpeed(unsigned char speed);
	unsigned char setServoSpeed(byte servoNum, unsigned char speed);
	unsigned int getServoAnalogData(byte servoNum);
	unsigned char xyzToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight, boolean allowApproximate = true);
	unsigned char limitRange(double& angleRot, double& angleLeft, double& angleRight);
	double analogToAngle(byte servoNum, int inputAnalog);
	void translateCoordiante(double& x, double& y, double& z, bool reverse);

	bool checkLinearCalibrationFlag();
	bool checkOffsetFlag();
	bool checkAdcCalibrationFlag();
	bool servoAttached(byte servoNum);

	void powerUp();
	void saveCount();
	unsigned long getCountValue(int index);
private:
	double readServoAngleOffset(byte servoNum);

	
	void readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal);

	void sort(unsigned int array[], unsigned int len);

public:
	unsigned int mMaxAdcPos[SERVO_COUNT] = {180};

protected:
	Servo mServo[SERVO_COUNT];
	uint8_t mCoordinates = 1;

	unsigned char mServoSpeed = 255;
	double mCurAngle[SERVO_COUNT] = {90, 90, 0, 90};

	bool mGetLinearCalibrationFlag = false;
	bool mGetAdcCalibrationFlag = false;

	//offset of assembling
	double mServoAngleOffset[SERVO_COUNT];

	unsigned long mCount[SERVO_COUNT+1];
	unsigned long mLastCount[SERVO_COUNT+1];

	const byte SERVO_CONTROL_PIN[SERVO_COUNT] = {SERVO_ROT_PIN, SERVO_LEFT_PIN, SERVO_RIGHT_PIN, SERVO_HAND_ROT_PIN};
	const byte SERVO_ANALOG_PIN[SERVO_COUNT] = {SERVO_ROT_ANALOG_PIN, SERVO_LEFT_ANALOG_PIN, SERVO_RIGHT_ANALOG_PIN, SERVO_HAND_ROT_ANALOG_PIN};

};


extern uArmController controller;

#endif // _UARMCONTROLLER_H_
