/**
  ******************************************************************************
  * @file	uArmController.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmController.h" 
#include "uArmIIC.h"
#include "uArmRecorder.h"
#include "uArmAPI.h"

uArmController controller;

uArmController::uArmController()
{
	mServoSpeed = 255;
	mGetLinearCalibrationFlag = false;
	mGetAdcCalibrationFlag = false;

#ifdef SWIFT
	mCoordinates = 1;
#else
	mCoordinates = 0;
#endif
}

void uArmController::init()
{




#ifdef CALIBRATION
	mServoAngleOffset[0] = 0;
	mServoAngleOffset[1] = 0;
	mServoAngleOffset[2] = 0;
	mServoAngleOffset[3] = 0;

	if (checkLinearCalibrationFlag())
	{
		mGetLinearCalibrationFlag = true;
	}


	if (checkAdcCalibrationFlag())
	{

		mGetAdcCalibrationFlag = true;
	}
	

	for (int i = 0; i <= SERVO_COUNT; i++)
	{
		EEPROM.put(EERPOM_COUNT_ADDR+i*4, 0);
	}	
#else
	
	if (true)//checkOffsetFlag())
	{
		checkOffsetFlag();
		debugPrint("checkOffsetFlag\r\n");
		for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
		{
			double servoOffset = 0;

			servoOffset = readServoAngleOffset(i);
	        debugPrint("servoOffset=%f\r\n", servoOffset);
			if (isnan(servoOffset))
			{
				servoOffset = 0;
				setE2PROMData(EEPROM_EXTERN_SYSTEM, MANUAL_OFFSET_ADDRESS + i*sizeof(servoOffset), DATA_TYPE_FLOAT, servoOffset);

				//EEPROM.put(MANUAL_OFFSET_ADDRESS + i * sizeof(servoOffset), servoOffset);
			}
	        
	        mServoAngleOffset[i] = servoOffset;

		}
	}
	else
	{
		debugPrint("checkOffsetFlag Fail\r\n");
		mServoAngleOffset[0] = 0;
		mServoAngleOffset[1] = 0;
		mServoAngleOffset[2] = 0;
		mServoAngleOffset[3] = 0;		
	}

	// check calibration flag
	if (checkLinearCalibrationFlag())
	{
		mGetLinearCalibrationFlag = true;
	}


	if (checkAdcCalibrationFlag())
	{

		mGetAdcCalibrationFlag = true;
	}
#endif


	for (int k = 0; k < 3; k++)
	{

		delay(10);
		unsigned char data[2];
		unsigned int offset = ADC_START_ADDRESS + k * 1024 + 500;

		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);

		mMaxAdcPos[k] = (data[0] << 8) + data[1];

		//Serial.println(mMaxAdcPos[k]);

	}


	mServo[SERVO_ROT_NUM].setPulseWidthRange(500, 2500);
	mServo[SERVO_LEFT_NUM].setPulseWidthRange(500, 2500);
	mServo[SERVO_RIGHT_NUM].setPulseWidthRange(500, 2500);
	mServo[SERVO_HAND_ROT_NUM].setPulseWidthRange(600, 2400);

	attachAllServo();  

#ifndef CALIBRATION
	// writeServoAngle(SERVO_ROT_NUM, 90);
	// writeServoAngle(SERVO_LEFT_NUM, 90);
	// writeServoAngle(SERVO_RIGHT_NUM, 0);
	// writeServoAngle(SERVO_HAND_ROT_NUM, 90);   
	mCurAngle[0] = readServoAngle(SERVO_ROT_NUM, true);
	mCurAngle[1] = readServoAngle(SERVO_LEFT_NUM, true);
	mCurAngle[2] = readServoAngle(SERVO_RIGHT_NUM, true);
	mCurAngle[3] = readServoAngle(SERVO_HAND_ROT_NUM, true);
#endif

	for (int i = 0; i <= SERVO_COUNT; i++)
	{
		EEPROM.get(EERPOM_COUNT_ADDR+i*4, mCount[i]);
		mLastCount[i] = mCount[i];
	}

}

unsigned long uArmController::getCountValue(int index)
{
	if (index > SERVO_COUNT)
		return 0;

	return mCount[index];
}

void uArmController::powerUp()
{	
	mCount[SERVO_COUNT]++;
}

void uArmController::saveCount()
{
	for (int i = 0; i <= SERVO_COUNT; i++)
	{
		if (mCount[i] != mLastCount[i])
		{
			mLastCount[i] = mCount[i];
			EEPROM.put(EERPOM_COUNT_ADDR+i*4, mCount[i]);
		}
	}
}

bool uArmController::checkLinearCalibrationFlag()
{
	uint16_t offset = 0;
	uint8_t data[2];
	uint16_t value1 = 0, value2 = 0xffff;

	for (int i = 0; i < 3; i++)
	{
		offset = ADC_START_ADDRESS + i * 1024 + 500;
		delay(10);
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);
		value1 = (data[0] << 8) + data[1];

		// add linear calibratoin flag
		delay(10);
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, LINEAR_CALIBRATION_FLAG_ADDREESS+i*2, 2);
		value2 = (data[0] << 8) + data[1];

		if (value1 != value2)
			return false;
	}
	return true;
}

bool uArmController::checkAdcCalibrationFlag()
{
	uint16_t offset = 0;
	uint8_t data[2];
	uint16_t value1 = 0, value2 = 0xffff;

	for (int i = 0; i < 3; i++)
	{
		offset = ADC_START_ADDRESS + i * 1024 + ADC_FALG_DATA_INDEX*2;
		delay(10);
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);
		value1 = (data[0] << 8) + data[1];

		// add linear calibratoin flag
		delay(10);
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, ADC_CALIBRATION_FLAG_ADDRESS+i*2, 2);
		value2 = (data[0] << 8) + data[1];

		if (value1 != value2)
			return false;
	}
	return true;
}

bool uArmController::checkOffsetFlag()
{

	double offset = 0;
	double offset2 = 0;
	for (int i = 0; i < 4; i++)
	{
		delay(5);
		offset = getE2PROMData(EEPROM_EXTERN_SYSTEM, MANUAL_OFFSET_ADDRESS + i*sizeof(offset), DATA_TYPE_FLOAT);			
		//debugPrint("offset1=%f\r\n", offset);
		//if (offset < 0 || offset > MAX_MANUAL_OFFSET)
		//	return false;

		delay(5);
		offset2 = getE2PROMData(EEPROM_EXTERN_SYSTEM, OFFSET_CALIBRATION_FLAG_ADDRESS + i*sizeof(offset2), DATA_TYPE_FLOAT);			
		//debugPrint("offset2=%f\r\n", offset2);
		//if (offset != offset2)
		//	return false;

	}
	return true;
}

void uArmController::attachAllServo()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
		mServo[i].attach(SERVO_CONTROL_PIN[i]);
	}
}

void uArmController::attachServo(byte servoNum)
{
	mServo[servoNum].attach(SERVO_CONTROL_PIN[servoNum]);
}

void uArmController::detachServo(byte servoNum)
{
	mServo[servoNum].detach();
}

void uArmController::detachAllServo()
{
	for (int i = SERVO_ROT_NUM; i < SERVO_COUNT; i++)
	{
		detachServo(i);
	}
}

void uArmController::writeServoAngle(double servoRotAngle, double servoLeftAngle, double servoRightAngle)
{
	writeServoAngle(SERVO_ROT_NUM, servoRotAngle);
	writeServoAngle(SERVO_LEFT_NUM, servoLeftAngle);
	writeServoAngle(SERVO_RIGHT_NUM, servoRightAngle);
}

double uArmController::getReverseServoAngle(byte servoNum, double servoAngle)
{
#if defined(MKII) || defined(SWIFT)

	if (servoAngle < mCurAngle[servoNum])
	{
		unsigned char data[2];

		int i_val = 0;
		int addr = ADC_START_ADDRESS + servoNum * 1024 + 512 + servoAngle * 2;

		addr &= 0xfffe;
		
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, addr, 2);

		i_val = (data[0] << 8) + data[1];
		

		if (abs(i_val) > 3)
			return servoAngle;

		servoAngle = servoAngle - (i_val) - 1;
	}
#endif
	return servoAngle;

}

void uArmController::writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset )
{


#ifdef CALIBRATION


    if (writeWithOffset)
    {
        switch (servoNum)
        {
        case SERVO_ROT_NUM:
            readServoCalibrationData(ROT_SERVO_ADDRESS, servoAngle);
            break;

        case SERVO_LEFT_NUM:
            readServoCalibrationData(LEFT_SERVO_ADDRESS, servoAngle);
            break;

        case SERVO_RIGHT_NUM:
            readServoCalibrationData(RIGHT_SERVO_ADDRESS, servoAngle);
            break;

        }        
    }

	mServo[servoNum].write(servoAngle);
#else

	servoAngle = constrain(servoAngle, 0.00, 180.00);

	mCurAngle[servoNum] = servoAngle;
	servoAngle = writeWithOffset ? (servoAngle + mServoAngleOffset[servoNum]) : servoAngle;

#if (defined(MKII) || defined(SWIFT))
	
	if (mGetAdcCalibrationFlag)
	{
		servoAngle = getReverseServoAngle(servoNum, servoAngle);
	}

	if (mGetLinearCalibrationFlag)
	{
		switch (servoNum)
		{
		case SERVO_ROT_NUM:
			readServoCalibrationData(ROT_SERVO_ADDRESS, servoAngle);
			break;

		case SERVO_LEFT_NUM:
			readServoCalibrationData(LEFT_SERVO_ADDRESS, servoAngle);
			break;

		case SERVO_RIGHT_NUM:
			readServoCalibrationData(RIGHT_SERVO_ADDRESS, servoAngle);
			break;

		}
	}
#endif


	mServo[servoNum].write(servoAngle, mServoSpeed);

	if (isPowerPlugIn())
	{
		mCount[servoNum]++;
	}

#endif
}


#if defined(MKII) || defined(SWIFT)
void uArmController::readServoCalibrationData(unsigned int address, double& angle)
{
	unsigned char calibration_data[DATA_LENGTH]; //get the calibration data around the data input
	unsigned int min_data_calibration_address;
	double closest_data, another_closest_data;
	unsigned int deltaA = 0xffff, deltaB = 0, i, i_min = 0;
	deltaA = 0xffff;
	deltaB = 0;

	//debugPrint("angle in =%f\r\n", angle);

	if (abs(angle - 0.0) < 0.001)
	{
		return;
	}

	if (angle < (DATA_LENGTH >> 2))
	{
		min_data_calibration_address = 0;
	}
	else if (angle > (180 - (DATA_LENGTH >> 2)))
	{
		min_data_calibration_address = (((unsigned int)180 - (DATA_LENGTH >> 1)) * 2);
	}
	else
	{
		min_data_calibration_address = (((unsigned int)angle - (DATA_LENGTH >> 2)) * 2);
	}


	unsigned char dataLen = DATA_LENGTH;
	if (min_data_calibration_address + dataLen > 360)
	{
		dataLen = 360 - min_data_calibration_address;
	}


	//delay(5);
	iic_readbuf(calibration_data, EXTERNAL_EEPROM_SYS_ADDRESS, address + min_data_calibration_address, dataLen);

	// for (i = 0; i <  (DATA_LENGTH >> 1); i++)
	// {
	// 	delay(5);
	// 	int value = getE2PROMData(EEPROM_EXTERN_SYSTEM, address + min_data_calibration_address+i*2, DATA_TYPE_INTEGER);
	// 	//debugPrint("value[%d] = %d\r\n", i, value);

	// 	calibration_data[2*i] = (value & 0xff00) >> 8;
	// 	calibration_data[2*i+1] = value & 0xff;
	// }

#ifdef DEBUG
	// for (i = 0; i < (DATA_LENGTH >> 1); i++)
	// {
	// 	int value = (calibration_data[2*i]<<8) + calibration_data[2*i + 1];
	// 	debugPrint("value[%d] = %d\r\n", i, value);
	// }
#endif
	for(i=0;i<(dataLen >> 1);i++)
	{
		deltaB = abs ((calibration_data[i+i]<<8) + calibration_data[1+(i+i)] - angle * 10);
		if(deltaA > deltaB)
		{
			i_min = i;
			deltaA = deltaB;
		}
	}

	closest_data = ((calibration_data[i_min+i_min]<<8) + calibration_data[1+(i_min+i_min)])/10.0;//transfer the dat from ideal data to servo angles
	if(angle >= closest_data)
	{
		if (i_min == 0 || (i_min > 0 && (i_min-1)* 2 < dataLen))
		{
			another_closest_data = ((calibration_data[i_min+i_min+2]<<8) + calibration_data[3+i_min+i_min])/10.0;//bigger than closest



			if(abs(another_closest_data - closest_data) < 0.001)
			{
				angle = min_data_calibration_address/2 + i_min + 0.5;
			}
			else
			{
				angle = 1.0 * (angle - closest_data) / (another_closest_data - closest_data) + min_data_calibration_address/2 + i_min ;
			}
		}
		else
		{
			angle = min_data_calibration_address/2 + i_min + 0.5;
		}
	}
	else
	{
		if (i_min > 0)
		{
			another_closest_data = ((calibration_data[i_min+i_min-2]<<8) + calibration_data[i_min+i_min-1])/10.0;//smaller than closest


			if(abs(another_closest_data - closest_data) < 0.001)
			{
				angle = min_data_calibration_address/2 + i_min - 1 + 0.5;
			}
			else
			{
				angle = 1.0 * (angle - another_closest_data) / (closest_data - another_closest_data) + min_data_calibration_address/2 + i_min - 1;
			}
		}
		else
		{
			angle = min_data_calibration_address/2 + i_min + 0.5;
		}
	}    


	//debugPrint("angle=%f\r\n",angle);

}
#endif

double uArmController::readServoAngle(byte servoNum, boolean withOffset )
{
	double angle;

	if (mGetAdcCalibrationFlag)
	{
		if (servoNum == SERVO_HAND_ROT_NUM)
		{
			angle = map(getServoAnalogData(SERVO_HAND_ROT_ANALOG_PIN), SERVO_9G_MIN, SERVO_9G_MAX, 0, 180);
		}
		else
		{
			angle = analogToAngle(servoNum, getServoAnalogData(servoNum)); 
		}
	}
	else
	{

		if (servoNum == SERVO_HAND_ROT_NUM)
		{
			angle = map(getServoAnalogData(SERVO_HAND_ROT_ANALOG_PIN), SERVO_9G_MIN, SERVO_9G_MAX, 0, 180);
		}
		else
		{
			angle = map(getServoAnalogData(servoNum), SERVO_MIN, SERVO_MAX, 0, 180); 
		}		
	}
	

	if (withOffset)
	{
		angle -= mServoAngleOffset[servoNum];
	}

	angle = constrain(angle, 0.00, 180.00);

	return angle;
}


double uArmController::readServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle, boolean withOffset)
{
	servoRotAngle = readServoAngle(SERVO_ROT_NUM, withOffset);
	servoLeftAngle = readServoAngle(SERVO_LEFT_NUM, withOffset);
	servoRightAngle = readServoAngle(SERVO_RIGHT_NUM, withOffset);
}

bool uArmController::servoAttached(byte servoNum)
{
	return mServo[servoNum].attached();
}

double uArmController::getServoAngles(double& servoRotAngle, double& servoLeftAngle, double& servoRightAngle)
{

	servoRotAngle = mCurAngle[SERVO_ROT_NUM];
	servoLeftAngle = mCurAngle[SERVO_LEFT_NUM];
	servoRightAngle = mCurAngle[SERVO_RIGHT_NUM];

}

double uArmController::getServeAngle(byte servoNum)
{
	return mCurAngle[servoNum];
}

void uArmController::updateAllServoAngle(boolean withOffset)
{
	
	for (unsigned char servoNum = SERVO_ROT_NUM; servoNum < SERVO_COUNT; servoNum++)
	{
		mCurAngle[servoNum] = readServoAngle(servoNum, withOffset); 	
	}
	
}

unsigned char uArmController::getCurrentXYZ(double& x, double& y, double& z)
{

	// 在XY平面的投影长度
	double stretch = MATH_LOWER_ARM * cos(mCurAngle[SERVO_LEFT_NUM] / MATH_TRANS) + MATH_UPPER_ARM * cos(mCurAngle[SERVO_RIGHT_NUM] / MATH_TRANS) + MATH_L2 + MATH_FRONT_HEADER;

	// 在Z轴的投影长度,
	double height = MATH_LOWER_ARM * sin(mCurAngle[SERVO_LEFT_NUM] / MATH_TRANS) - MATH_UPPER_ARM * sin(mCurAngle[SERVO_RIGHT_NUM] / MATH_TRANS) + MATH_L1;
	x = stretch * cos(mCurAngle[SERVO_ROT_NUM] / MATH_TRANS);
	y = stretch * sin(mCurAngle[SERVO_ROT_NUM] / MATH_TRANS);
	z = height;


	translateCoordiante(x, y, z, true);
	

	return IN_RANGE;
}

void uArmController::translateCoordiante(double& x, double& y, double& z, bool reverse)
{
	if (mCoordinates)
	{
		double x_copy = x;
		double y_copy = y;
		if (reverse)
		{
			x = y_copy;
			y = -x_copy;
		}
		else
		{
			x = -y_copy;
			y = x_copy;
		}
	}
}

// unsigned char uArmController::getXYZFromPolar(double& x, double& y, double& z, double s, double r, double h)
// {
//     double stretch = s;

//     z = h;  
//     x = s * cos(r / MATH_TRANS);
//     y = s * sin(r / MATH_TRANS);
// }


unsigned char uArmController::getXYZFromAngle(double& x, double& y, double& z, double rot, double left, double right)
{
	// 在XY平面的投影长度
	double stretch = MATH_LOWER_ARM * cos(left / MATH_TRANS) + MATH_UPPER_ARM * cos(right / MATH_TRANS) + MATH_L2 + MATH_FRONT_HEADER;

	// 在Z轴的投影长度,
	double height = MATH_LOWER_ARM * sin(left / MATH_TRANS) - MATH_UPPER_ARM * sin(right / MATH_TRANS) + MATH_L1;
	x = stretch * cos(rot / MATH_TRANS);
	y = stretch * sin(rot / MATH_TRANS);
	z = height;


	translateCoordiante(x, y, z, true);


	return IN_RANGE;    
}

unsigned char uArmController::xyzToAngle(double x, double y, double z, double& angleRot, double& angleLeft, double& angleRight, boolean allowApproximate)
{
	double xIn = 0.0;
	double zIn = 0.0;
	double rightAll = 0.0;
	double sqrtZX = 0.0;
	double phi = 0.0;

	x = constrain(x,-3276,3276);
	y = constrain(y,-3276,3276);
	z = constrain(z,-3276,3276);
	x = (double)((int)(x*10)/10.0);
	y = (double)((int)(y*10)/10.0);
	z = (double)((int)(z*10)/10.0);


	translateCoordiante(x, y, z, false);
	

	if (z > MAX_Z || z < MIN_Z)
	{
		return OUT_OF_RANGE_NO_SOLUTION;
	}

	zIn = (z - MATH_L1) / MATH_LOWER_ARM;

	if(!allowApproximate)//if need the move to closest point we have to jump over the return function
	{
	//check the range of x
		if(y<0)
		{
			return OUT_OF_RANGE_NO_SOLUTION;
		}
	}

	// Calculate value of theta 1: the rotation angle
	if (x == 0)
	{
		angleRot = 90;
	}
	else
	{
		if (x > 0)
		{
			angleRot = atan(y / x) * MATH_TRANS;//angle tranfer 0-180 CCW 弧度转为角度
		}
		if (x < 0)
		{
			angleRot = 180 + atan(y / x) * MATH_TRANS;//angle tranfer  0-180 CCW
		}
	}

    	// Calculate value of theta 3
	if(angleRot != 90)//xIn is the stretch
	{
		xIn = (x / cos(angleRot / MATH_TRANS) - MATH_L2 - MATH_FRONT_HEADER) / MATH_LOWER_ARM;
	}
	else
	{
		xIn = (y - MATH_L2 - MATH_FRONT_HEADER) / MATH_LOWER_ARM;
	}   

	phi = atan(zIn / xIn) * MATH_TRANS;//phi is the angle of line (from joint 2 to joint 4) with the horizon

	sqrtZX = sqrt(zIn*zIn + xIn*xIn); // 节点2到节点4的长度

	rightAll = (sqrtZX*sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER  - 1) / (2 * MATH_UPPER_LOWER  * sqrtZX);//cosin law
	angleRight = acos(rightAll) * MATH_TRANS;//cosin law

	// Calculate value of theta 2
	rightAll = (sqrtZX*sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER ) / (2 * sqrtZX);//cosin law
	angleLeft = acos(rightAll) * MATH_TRANS;//cosin law

	angleLeft = angleLeft + phi;
	angleRight = angleRight - phi;

	//determine if the angle can be reached
	return limitRange(angleRot, angleLeft, angleRight);
}

unsigned char uArmController::limitRange(double& angleRot, double& angleLeft, double& angleRight)
{
	unsigned char result = IN_RANGE;

	//determine if the angle can be reached
	if(isnan(angleRot)||isnan(angleLeft)||isnan(angleRight))
	{
		result = OUT_OF_RANGE_NO_SOLUTION;
	}


	if (angleLeft > LOWER_ARM_MAX_ANGLE)
	{
		angleLeft = LOWER_ARM_MAX_ANGLE;
		result = OUT_OF_RANGE;
	}
	else if (angleLeft < LOWER_ARM_MIN_ANGLE)
	{
		angleLeft = LOWER_ARM_MAX_ANGLE;
		result = OUT_OF_RANGE;		
	}

	if (angleRight > UPPER_ARM_MAX_ANGLE)
	{
		angleRight = UPPER_ARM_MAX_ANGLE;
		result = OUT_OF_RANGE;
	}
	else if (angleRight < UPPER_ARM_MIN_ANGLE)
	{
		angleRight = UPPER_ARM_MIN_ANGLE;
		result = OUT_OF_RANGE;		
	}

	if (angleLeft + angleRight > (180 - LOWER_UPPER_MIN_ANGLE))
	{
		angleLeft = 180  - LOWER_UPPER_MIN_ANGLE - angleRight;
		result = OUT_OF_RANGE;
	}

	if ((180 - angleLeft - angleRight) > LOWER_UPPER_MAX_ANGLE)
	{
		angleRight = 180 - LOWER_UPPER_MAX_ANGLE - angleLeft;
		result = OUT_OF_RANGE;
	}

	angleRot = constrain(angleRot, 0.00, 180.00);
	angleLeft = constrain(angleLeft, 0.00, 180.00);
	angleRight = constrain(angleRight, 0.00, 180.00);


	return result;
}

unsigned char uArmController::validateAngle(byte srevoNum, double &angle)
{
	unsigned char result = IN_RANGE;

	switch (srevoNum)
	{
	case SERVO_ROT_NUM:
	case SERVO_HAND_ROT_NUM:
		angle = constrain(angle, 0.00, 180.00);
		break;

	case SERVO_LEFT_NUM:
	{
		double angleRight = getServeAngle(SERVO_RIGHT_NUM);
		if (angle > LOWER_ARM_MAX_ANGLE)
		{
			angle = LOWER_ARM_MAX_ANGLE;
			result = OUT_OF_RANGE;
		}
		else if (angle < LOWER_ARM_MIN_ANGLE)
		{
			angle = LOWER_ARM_MAX_ANGLE;
			result = OUT_OF_RANGE;		
		}	

		if (angle + angleRight > (180 - LOWER_UPPER_MIN_ANGLE))
		{
			angle = (180 - LOWER_UPPER_MIN_ANGLE) - angleRight;
			result = OUT_OF_RANGE;
		}

		if ((180 - angle - angleRight) > LOWER_UPPER_MAX_ANGLE)
		{
			angle = 180 - LOWER_UPPER_MAX_ANGLE - angleRight;
			result = OUT_OF_RANGE;
		}		

		break;
	}
	case SERVO_RIGHT_NUM:
	{

		double angleLeft = getServeAngle(SERVO_LEFT_NUM);
		if (angle > UPPER_ARM_MAX_ANGLE)
		{
			angle = UPPER_ARM_MAX_ANGLE;
			result = OUT_OF_RANGE;
		}
		else if (angle < UPPER_ARM_MIN_ANGLE)
		{
			angle = UPPER_ARM_MIN_ANGLE;
			result = OUT_OF_RANGE;		
		}

		if (angle + angleLeft > (180 - LOWER_UPPER_MIN_ANGLE))
		{
			angle = (180 - LOWER_UPPER_MIN_ANGLE) - angleLeft;
			result = OUT_OF_RANGE;
		}

		if ((180 - angle - angleLeft) > LOWER_UPPER_MAX_ANGLE)
		{
			angle = 180 - LOWER_UPPER_MAX_ANGLE - angleLeft;
			result = OUT_OF_RANGE;
		}		
		break;
	}

	default:
		return OUT_OF_RANGE_NO_SOLUTION;
		break;
	}

	return result;
}

void uArmController::readLinearOffset(byte servoNum, double& interceptVal, double& slopeVal)
{
	EEPROM.get(LINEAR_INTERCEPT_START_ADDRESS + servoNum * sizeof(interceptVal), interceptVal);
	EEPROM.get(LINEAR_SLOPE_START_ADDRESS + servoNum * sizeof(slopeVal), slopeVal);
}


#if defined(MKII) || defined(SWIFT)
double uArmController::analogToAngle(byte servoNum, int inputAnalog)
{


	int startAddr = ADC_START_ADDRESS + servoNum * 1024;
	// binary search
	bool done = false;
	int min = 0;
	
	int max = mMaxAdcPos[servoNum];

	int angle = (min + max) / 2;
	unsigned char data[2];
	int val = 0;

	debugPrint("inputAnalog=%d\r\n", inputAnalog);
	while (!done && (min < max))
	{
		//debugPrint("angle=%d", angle);
		//gRecorder.read(startAddr+angle*2, data, 2);
		
		iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, startAddr+angle*2, 2);
		
		val = (data[0] << 8) + data[1];
		//debugPrint("val=%d", val);


	#ifdef METAL_MOTOR
		if (val == inputAnalog)
		{
			return angle;
		}
		else if (val > inputAnalog)
		{
			min = angle;
		}
		else
		{
			max = angle;
		}
	#else
		if (val == inputAnalog)
		{
			return angle;
		}
		else if (val < inputAnalog)
		{
			min = angle;
		}
		else
		{
			max = angle;
		}        
	#endif

	angle = (min + max) / 2;

	//debugPrint("addr2=%d", angle);
	if (angle == min || angle == max)
		break;
	}   


	// Serial.print("angle=");
	// Serial.println(angle);

	if (angle == 0 || angle == mMaxAdcPos[servoNum])
	{
		return angle;
	}

	unsigned char adc_data[6];

	iic_readbuf(adc_data, EXTERNAL_EEPROM_SYS_ADDRESS, startAddr+(angle-1)*2, 6);

	min = (adc_data[0] << 8) + adc_data[1];
	int mid = (adc_data[2] << 8) + adc_data[3];
	max = (adc_data[4] << 8) + adc_data[5];

	double angleMin = 0.0;
	double angleMax = 0.0;
	double angleValue = 0;

	if (inputAnalog > min && inputAnalog <= mid)
	{
		angleMin = angle - 1;
		angleMax = angle;
		max = mid;
	}
	else
	{
		angleMin = angle;
		angleMax = angle+1;
		min = mid;
	}


    	angleValue = (inputAnalog - min) * (angleMax - angleMin) / (max - min) + angleMin ;


	// Serial.print("angleValue=");
	// Serial.println(angleValue);

    	return angleValue;

}

#elif defined(METAL)

double uArmController::analogToAngle(byte servoNum, int inputAnalog)
{
	double intercept = 0.0f;
	double slope = 0.0f;



	readLinearOffset(servoNum, intercept, slope);


	double angle = intercept + slope * inputAnalog;  




	return angle;
}
#endif


unsigned int uArmController::getServoAnalogData(byte servoNum)
{
	return getAnalogPinValue(SERVO_ANALOG_PIN[servoNum]);
}


double uArmController::readServoAngleOffset(byte servoNum)
{
	double manualServoOffset = 0.0f;

	//EEPROM.get(MANUAL_OFFSET_ADDRESS + servoNum * sizeof(manualServoOffset), manualServoOffset);
	//delay(5);
	manualServoOffset = getE2PROMData(EEPROM_EXTERN_SYSTEM, MANUAL_OFFSET_ADDRESS + servoNum*sizeof(manualServoOffset), DATA_TYPE_FLOAT);			


	return manualServoOffset;	
}

unsigned char uArmController::setServoSpeed(byte servoNum, unsigned char speed)
{
	mServoSpeed = speed;
}

unsigned char uArmController::setServoSpeed(unsigned char speed)
{
	setServoSpeed(SERVO_ROT_NUM, speed);
	setServoSpeed(SERVO_LEFT_NUM, speed);
	setServoSpeed(SERVO_RIGHT_NUM, speed);
}