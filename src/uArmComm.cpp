/**
  ******************************************************************************
  * @file	gCommComm.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmComm.h" 
//#include "uArmRingBuffer.h"


static CommState commState = IDLE;
static unsigned char cmdReceived[COM_LEN_MAX];
static unsigned char cmdIndex = 0;

static HardwareSerial* CommSerial = NULL;
//static uArmRingBuffer ringBuffer;

#define RESULT_BUFFER_SIZE  50

static unsigned char comLock = false;
// #define RING_BUFFER_SIZE    48
// uint8_t bufData[RING_BUFFER_SIZE];

typedef struct _MoveCmd
{
	uint16_t serialNum;
	uint16_t cmdCode;
	double parameters[4];
	uint8_t parameterCount;
} MoveCmd_t;

#ifdef SWIFT
#define MOVE_CMD_BUFFER_SIZE	20
#else
#define MOVE_CMD_BUFFER_SIZE	2
#endif

class MoveCmdBuffer
{
public:
	MoveCmdBuffer();
	uint8_t put(MoveCmd_t *cmd);
	uint8_t get(MoveCmd_t *cmd);
	uint8_t isFull();
	uint8_t isEmpty();
	void clear();

private:
	MoveCmd_t cmds[MOVE_CMD_BUFFER_SIZE];
	uint8_t tail;
	uint8_t head;
	uint8_t count;
};

MoveCmdBuffer::MoveCmdBuffer()
{
	head = 0;
	tail = 0;
	count = 0;
}

void MoveCmdBuffer::clear()
{
	head = 0;
	tail = 0;
	count = 0;
}

uint8_t MoveCmdBuffer::put(MoveCmd_t *cmd)
{
	if (isFull())
	{
		return 0;
	}

	count++;

	cmds[tail].serialNum = cmd->serialNum;
	cmds[tail].cmdCode = cmd->cmdCode;
	cmds[tail].parameterCount = cmd->parameterCount;

	for (int i = 0; i < cmds[tail].parameterCount; i++)
	{
		cmds[tail].parameters[i] = cmd->parameters[i];
	}
	tail++;
	tail = tail % MOVE_CMD_BUFFER_SIZE;

	return 1;
}

uint8_t MoveCmdBuffer::get(MoveCmd_t *cmd)
{
	if (isEmpty())
	{
		return 0;
	}

	cmd->serialNum = cmds[head].serialNum;
	cmd->cmdCode = cmds[head].cmdCode;
	cmd->parameterCount = cmds[head].parameterCount;

	for (int i = 0; i < cmds[head].parameterCount; i++)
	{
		cmd->parameters[i] = cmds[head].parameters[i];
	}
	head++;
	head = head % MOVE_CMD_BUFFER_SIZE;	

	count--;

	return 1;
}

uint8_t MoveCmdBuffer::isFull()
{
	if (count >= MOVE_CMD_BUFFER_SIZE)
	{
		return 1;
	}

	return 0;

}

uint8_t MoveCmdBuffer::isEmpty()
{
	if (count == 0)
	{
		return 1;
	}

	return 0;
}



MoveCmdBuffer moveCmdBuffer;

static void replyError(int serialNum, unsigned int errorCode)
{
	if (serialNum > 0)
	{
		CommSerial->print("$");
		CommSerial->print(serialNum);
		CommSerial->print(" ");
	}

	CommSerial->print("E");
	CommSerial->println(errorCode);   
}

static void replyOK(int serialNum)
{
	if (serialNum > 0)
	{
		CommSerial->print("$");
		CommSerial->print(serialNum);
		CommSerial->print(" ");
	}
	CommSerial->println("OK");   
}

static void replyResult(int serialNum, String result)
{
	if (serialNum > 0)
	{    
		CommSerial->print("$");
		CommSerial->print(serialNum);
		CommSerial->print(" ");
	}
	CommSerial->print("OK ");
	CommSerial->println(result);   
}

static void reportResult(int reportCode, String result)
{

	CommSerial->print("@");
	CommSerial->print(reportCode);
	CommSerial->print(" ");
	CommSerial->println(result);   
}

void reportString(String string)
{
	CommSerial->println(string);
}

static unsigned char cmdMove(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 4)
		return PARAMETER_ERROR;
 
	unsigned char result = moveTo(value[0], value[1], value[2], value[3]);

	if (result == ERR_NO_POWER_CONNECTED)
	{
		return ERR_NO_POWER;
	}
	else if (result != OUT_OF_RANGE_NO_SOLUTION)
	{
		replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

	return 0;
}

static unsigned char cmdMovePol(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 4)
		return PARAMETER_ERROR;
	
	unsigned char result = moveToPol(value[0], value[1], value[2], value[3]);

	if (result == ERR_NO_POWER_CONNECTED)
	{
		return ERR_NO_POWER;
	}
	else if (result != OUT_OF_RANGE_NO_SOLUTION)
	{
		replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

	return 0;   
}

static unsigned char cmdSetAttachServo(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 1)
		return PARAMETER_ERROR;

	if (attachServo(value[0]))
	{
		replyOK(serialNum);
		return 0;
	}
	else
	{
		return OUT_OF_RANGE;
	}
  
}

static unsigned char cmdSetDetachServo(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	if (detachServo(value[0]))
	{
		replyOK(serialNum);
		return 0;
	}
	else
	{
		return OUT_OF_RANGE;
	}

	return 0;
}

static unsigned char cmdIsServoAttached(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;	
	char result[RESULT_BUFFER_SIZE];

	replyResult(serialNum, result);
	if (isServoAttached(value[0]))
	{
		strcpy(result, "V1");
	}
	else
	{
		strcpy(result, "V0");
	}

	return 0;
}

// static unsigned char cmdSetServoAngle(int serialNum, int parameterCount, double value[4])
// {
//     if (parameterCount != 2)
//         return PARAMETER_ERROR;

// 	uArm.mController.writeServoAngle(byte(value[0]), value[1], false);
//     replyOK(serialNum);

//     return 0;
// }

static unsigned char cmdSetServoAngleWithOffset(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 2 && parameterCount != 3)
		return PARAMETER_ERROR;    

	double speed = 100;
	//debugPrint("N%f, V%f, count=%d\r\n", value[0], value[1], parameterCount);

	if (parameterCount == 3)
	{
		speed = value[2];
	}

	if (setServoAngle(byte(value[0]), value[1], speed) == OK)
	{
		replyOK(serialNum);
		return 0;
	}
	else
	{
		return PARAMETER_ERROR; 
	}

    

}

static unsigned char cmdSetPump(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	if (value[0] == 0)//off
	{
		pumpOff();
	}
	else//on
	{
		pumpOn();
	}

	replyOK(serialNum);

	return 0;
}



static unsigned char cmdSetMacEnable(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;	

	replyOK(serialNum);

	service.disableBT(value[0] ? false : true);

	return 0;	
}

static unsigned char cmdSetClearMacFlag(int serialNum, int parameterCount, double value[4])
{

	clearMacFlag();

	replyOK(serialNum);

	return 0;
}

static unsigned char cmdSetGripper(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 1)
		return PARAMETER_ERROR;

	if (value[0]==0)//release
	{
		gripperRelease();
	}
	else//catch
	{
		gripperCatch();
	}

	replyOK(serialNum);

	return 0;
}

static unsigned char cmdSetBuzz(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 2)
		return PARAMETER_ERROR;

	buzzer.buzz(value[0], value[1]);    // convert to ms

	replyOK(serialNum);

	return 0;
}


static unsigned char cmdStopMove(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	stopMove();
	buzzer.stop();
	replyOK(serialNum);

	return 0;
}


static unsigned char cmdGetHWVersion(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;    

	char result[RESULT_BUFFER_SIZE];

#ifdef SWIFT
	msprintf(result, "V%s.%d", HW_VER, getHWSubversion());
#else
	msprintf(result, "V%s", HW_VER);
#endif


	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetSWVersion(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;    

	char result[RESULT_BUFFER_SIZE];

	msprintf(result, "V%s", SW_VER);


	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdSimulatePos(int serialNum, int parameterCount, double value[4])
{


	if (parameterCount != 4)
		return PARAMETER_ERROR;

	if (value[3] == 1) //Polar
	{
		double s = value[0];
		double r = value[1];
		double h = value[2];

		polToXYZ(s, r, h, value[0], value[1], value[2]);
	}

	unsigned char status = validatePos(value[0], value[1], value[2]);


	char result[RESULT_BUFFER_SIZE];
	switch(status)
	{
	case IN_RANGE: 
		strcpy(result, "V1");
	break;

	case OUT_OF_RANGE: 
	case OUT_OF_RANGE_NO_SOLUTION: 
		strcpy(result, "V0");
		break;
	default:                
		break;
	}

	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetCurrentXYZ(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 0)
		return PARAMETER_ERROR;

	getCurrentXYZ(value[0], value[1], value[2]);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

	replyResult(serialNum, result);
	return 0;	
}

static unsigned char cmdGetCurrentPosPol(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	getCurrentPosPol(value[0], value[1], value[2]);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "S%f R%f H%f", value[0], value[1], value[2]);

	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetCurrentAngle(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	value[0] = getServoAngle(SERVO_ROT_NUM);
	value[1] = getServoAngle(SERVO_LEFT_NUM);
	value[2] = getServoAngle(SERVO_RIGHT_NUM);
	value[3] = getServoAngle(SERVO_HAND_ROT_NUM);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "B%f L%f R%f H%f", value[0], value[1], value[2], value[3]);

	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetCurrentMotorAngle(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	unsigned char servoIndex = value[0];

	if (servoIndex > 3)
		return PARAMETER_ERROR;

	double angle = getServoAngle(servoIndex);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "V%f", angle);

	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdCoordinateToAngle(int serialNum, int parameterCount, double value[4])
{
	double rot, left, right;

	if (parameterCount != 3)
		return PARAMETER_ERROR;

	xyzToAngle(value[0], value[1], value[2], rot, left, right);

	value[0] = rot;
	value[1] = left;
	value[2] = right;

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "B%f L%f R%f", value[0], value[1], value[2]);

	replyResult(serialNum, result);


	return 0;
}

static unsigned char cmdAngleToXYZ(int serialNum, int parameterCount, double value[4])
{
	double x, y, z;
	bool success;

	if (parameterCount != 3)
		return PARAMETER_ERROR;

	if(angleToXYZ(value[0], value[1], value[2], x, y, z) == OUT_OF_RANGE)
	{
		success = false;
	}
	else
	{
		success = true;
	}

	value[0] = x;
	value[1] = y;
	value[2] = z;

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

	replyResult(serialNum, result);
	return 0;
}

#if defined(MKII) || defined(SWIFT)  
static unsigned char cmdIsMoving(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];
	if(isMoving())
	{
		strcpy(result, "V1");
	}
	else
	{
		strcpy(result, "V0");
	}

	replyResult(serialNum, result);

	return 0;
}
#endif

static unsigned char cmdGetTip(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];


	if(getTip())
	{
		strcpy(result, "V1");
	}
	else
	{
		strcpy(result, "V0");
	}

	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetDigitValue(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	int val = getDigitalPinValue(value[0]);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "V%d", val);

	replyResult(serialNum, result);
	return 0;
}

static unsigned char cmdSetBTName(char btName[])
{
	int serialNum = 0;
	
	if (setBtName(btName))
	{
		replyOK(serialNum);
	}
	else
	{
		replyError(serialNum, ERR_FAIL);
	}
	return 0;
}

static unsigned char cmdSetDigitValue(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 2)
		return PARAMETER_ERROR;

	setDigitalPinValue(value[0], value[1]);

	replyOK(serialNum);
	return 0;
}

extern void ResetBTUUID();
static unsigned char cmdResetBTUUID(int serialNum, int parameterCount, double value[4])
{

	ResetBTUUID();
	replyOK(serialNum);
	return 0;
}

static unsigned char cmdInitGroveModule(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	initGroveModule((GroveType)value[0]);

	replyOK(serialNum);

	return 0;
}

static unsigned char cmdSetGroveModuleReportInterval(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 2)
		return PARAMETER_ERROR;

	setGroveModuleReportInterval((GroveType)value[0], value[1]);

	replyOK(serialNum);

	return 0;
}

static unsigned char cmdGetCountValue(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;	

	unsigned long val = controller.getCountValue(value[0]);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "V%l", val);

	replyResult(serialNum, result);
	return 0;	
}

static unsigned char cmdGetAnalogValue(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 1)
		return PARAMETER_ERROR;

	int val = getAnalogPinValue(value[0]);

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "V%d", val);

	replyResult(serialNum, result);
	return 0;
}

static unsigned char cmdGetE2PROMData(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 3)
		return PARAMETER_ERROR;    

	char result[RESULT_BUFFER_SIZE];

	int device = int(value[0]);
	int type = value[2];
	uint32_t addr = value[1];

	double resultVal = getE2PROMData(device, addr, type);

	switch(type)
	{
		case DATA_TYPE_BYTE:
		{
			int val = resultVal;
			msprintf(result, "V%d", val);
			break;
		}
		case DATA_TYPE_INTEGER:
		{
			int i_val = resultVal;
			msprintf(result, "V%d", i_val);
			break;
		}
		case DATA_TYPE_FLOAT:
		{
			double f_val = resultVal;
			msprintf(result, "V%f", f_val);
			break;
		}
	}

	replyResult(serialNum, result);    

	return 0;

}

static unsigned char cmdSetE2PROMData(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 4)
		return PARAMETER_ERROR;    



	int type = value[2];
	int device = int(value[0]);
	uint32_t addr = value[1];

	setE2PROMData(device, addr, type, value[3]);

	replyOK(serialNum);

	return 0;

}

static unsigned char cmdSetButtonService(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;

	if (value[0])
	{
		service.setButtonService(true); 
	}
	else
	{
		service.setButtonService(false); 
	}

	replyOK(serialNum);
	return 0;
}

static unsigned char cmdGetGripperStatus(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	unsigned char status = getGripperStatus();

	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "V%d", status);
	replyResult(serialNum, result);
	return 0;
}





static unsigned char cmdGetPumpStatus(int serialNum, int parameterCount, double value[4])
{

	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];

	unsigned char status = getPumpStatus();

#if defined(MKII) || defined(SWIFT)   
    
	msprintf(result, "V%d", status);

#elif defined(METAL)

	if (status)
		strcpy(result, "V1");
	else
		strcpy(result, "V0");

#endif

	replyResult(serialNum, result);

	return 0;
}


#if defined(MKII) || defined(SWIFT)  
static unsigned char cmdGetPowerStatus(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];

	if (isPowerPlugIn())
		strcpy(result, "V1");
	else
		strcpy(result, "V0");   

	replyResult(serialNum, result);

	return 0;
}
#endif 

// static unsigned char cmdGetServoAnalogData(int serialNum, int parameterCount, double value[4])
// {
//     if (parameterCount != 1)
//         return PARAMETER_ERROR;

//     unsigned int data = uArm.mController.getServoAnalogData(value[0]);
//     //CommSerial->println(data);
//     char result[RESULT_BUFFER_SIZE];


//     msprintf(result, "V%d", result);   

//     replyResult(serialNum, result);

//     return 0;
// }


static unsigned char cmdRelativeMove(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 4)
		return PARAMETER_ERROR;

	unsigned char result = relativeMove(value[0], value[1], value[2], value[3]);

	if (result == ERR_NO_POWER_CONNECTED)
	{
		return ERR_NO_POWER;
	}
	else if (result != OUT_OF_RANGE_NO_SOLUTION)
	{
		replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

	return 0;
}

static unsigned char cmdRelativeMovePol(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 4)
		return PARAMETER_ERROR;

	unsigned char result = relativeMovePol(value[0], value[1], value[2], value[3]);

	if (result == ERR_NO_POWER_CONNECTED)
	{
		return ERR_NO_POWER;
	}
	else if (result != OUT_OF_RANGE_NO_SOLUTION)
	{
		replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

	return 0;
}


static unsigned char cmdSetReportInterval(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 1)
		return PARAMETER_ERROR;


	service.setReportInterval(value[0]*1000);

	replyOK(serialNum);


	return 0;
}

static unsigned char cmdGetDeviceName(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];

	msprintf(result, "V%s", DEVICE_NAME);


	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetAPIVersion(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;

	char result[RESULT_BUFFER_SIZE];

	msprintf(result, "V%s", SW_VER);


	replyResult(serialNum, result);

	return 0;
}

static unsigned char cmdGetDeviceUUID(int serialNum, int parameterCount, double value[4])
{
	if (parameterCount != 0)
		return PARAMETER_ERROR;


	char result[RESULT_BUFFER_SIZE];

#ifdef SWIFT 
	msprintf(result, "V%s", getMac());
#else
	strcpy(result, "V1234567890");   
#endif

	replyResult(serialNum, result);

	return 0;
}

#ifdef CALIBRATION
//extern bool waitReady;
extern void encoderReady(unsigned char encoderUsed);
static unsigned char cmdWaitReady(int serialNum, int parameterCount, double value[4])
{

	//waitReady = false;
	encoderReady(value[0]);
	return 0;
}
extern void updateServoAngleData(int servoNum, int index, int data);
static unsigned char cmdGetEncoderData(int serialNum, int parameterCount, double value[4])
{

	updateServoAngleData(value[0], value[1], value[2]);
	return 0;
}	


extern void getTestResult(uint8_t type, uint8_t result[]);
static unsigned char cmdGetTestData(int serialNum, int parameterCount, double value[4])
{
	Serial.println("cmdGetTestData");
	char result[RESULT_BUFFER_SIZE];

	getTestResult(value[0], result);

	replyResult(serialNum, result);
	return 0;
	
}	
#endif

void reportButtonEvent(unsigned char buttonId, unsigned char event)
{
	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "B%d V%d", buttonId, event); 
	reportResult(REPORT_BUTTON, result);  
}


void reportPos()
{
	double x, y, z, frontEndAngle;

	getCurrentXYZ(x, y, z);
	frontEndAngle = getServoAngle(SERVO_HAND_ROT_NUM);

	debugPrint("angle = %f\r\n", frontEndAngle);
	char result[RESULT_BUFFER_SIZE];
	msprintf(result, "X%f Y%f Z%f R%f", x, y, z, frontEndAngle);   

	reportResult(REPORT_POS, result);    

}

static void HandleMoveCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
	unsigned char result = false;

	switch (cmdCode)
	{
	case 0:
		result = cmdMove(serialNum, parameterCount, value);
		break;

	case 201:
	case 2201:
		result = cmdMovePol(serialNum, parameterCount, value);
		break;

	case 202:
	case 2202:
		result = cmdSetServoAngleWithOffset(serialNum, parameterCount, value);
		break;

	case 203:
	case 2203:
		result = cmdStopMove(serialNum, parameterCount, value);
		break;

	case 204:
	case 2204:
		result = cmdRelativeMove(serialNum, parameterCount, value);
		break;

	case 2205:
		result = cmdRelativeMovePol(serialNum, parameterCount, value);
		break;		

	default:
		replyError(serialNum, NO_SUCH_CMD);
		return;
	}

	if (result > 0)
	{
		CommSerial->print("$");
		CommSerial->print(serialNum);
		CommSerial->print(" ");
		CommSerial->print("E");
		CommSerial->println(result);
	}
}

static void bufferMoveCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
	MoveCmd_t cmd;

	cmd.serialNum = serialNum;
	cmd.cmdCode = cmdCode;
	cmd.parameterCount = parameterCount;



	for (int i = 0; i < parameterCount; i++)
	{
		cmd.parameters[i] = value[i];
		//debugPrint("value[%d]=%f\r\n", i, value[i]);
	}

	if (!moveCmdBuffer.put(&cmd))
	{
		// buffer full;
		replyError(serialNum, ERR_BUFFER_FULL);
	}
}

static void HandleSettingCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
	unsigned char result = false;

	debugPrint("cmdCode = %d\r\n", cmdCode);

	switch (cmdCode)
	{
	case 120:
	case 2120:
		result = cmdSetReportInterval(serialNum, parameterCount, value);
		break;

#if defined(MKII) || defined(SWIFT)  
	case 200:
	case 2200:
		result = cmdIsMoving(serialNum, parameterCount, value);
		break;
#endif

	case 201:
	case 2201:
		result = cmdSetAttachServo(serialNum, parameterCount, value);
		break;

	case 202:
	case 2202:
		result = cmdSetDetachServo(serialNum, parameterCount, value);
		break;

	case 2203:
		result = cmdIsServoAttached(serialNum, parameterCount, value);
		break;		

	case 210:
	case 2210:
		result = cmdSetBuzz(serialNum, parameterCount, value);
		break;        

	case 211:
	case 2211:
		result = cmdGetE2PROMData(serialNum, parameterCount, value);
		break;

	case 212:
	case 2212:
		result = cmdSetE2PROMData(serialNum, parameterCount, value);
		break;

	case 213:
	case 2213:
		result = cmdSetButtonService(serialNum, parameterCount, value);
		break;        

	case 220:
	case 2220:
		result = cmdCoordinateToAngle(serialNum, parameterCount, value);
		break;

	case 221:
	case 2221:
		result = cmdAngleToXYZ(serialNum, parameterCount, value);
		break;

	case 222:
	case 2222:
		result = cmdSimulatePos(serialNum, parameterCount, value);
		break;

	case 231:
	case 2231:
		result = cmdSetPump(serialNum, parameterCount, value);
		break;

	case 232:
	case 2232:
		result = cmdSetGripper(serialNum, parameterCount, value);
		break;

	case 2233:
		result = cmdSetClearMacFlag(serialNum, parameterCount, value);
		break;

	case 2234:
		result = cmdSetMacEnable(serialNum, parameterCount, value);
		break;


	case 240:
	case 2240:
		result = cmdSetDigitValue(serialNum, parameterCount, value);
		break;
		
	case 2246:
		result = cmdResetBTUUID(serialNum, parameterCount, value);
		break;



#ifdef CALIBRATION
	case 250:
		cmdWaitReady(serialNum, parameterCount, value);
		result = 0;
		break;

	case 251:
		cmdGetEncoderData(serialNum, parameterCount, value);
		result = 0;
		break;

	case 260:
		cmdGetTestData(serialNum, parameterCount, value);
		result = 0;
		break;
#endif

	case 2300:
		result = cmdInitGroveModule(serialNum, parameterCount, value);
		break;

	case 2301:
		result = cmdSetGroveModuleReportInterval(serialNum, parameterCount, value);
		break;

	default:
		replyError(serialNum, NO_SUCH_CMD);
		return;
	}

	if (result > 0)
	{
		CommSerial->print("$");
		CommSerial->print(serialNum);
		CommSerial->print(" ");
		CommSerial->print("E");
		CommSerial->println(result);
	}
}

static void HandleQueryCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
	unsigned char result = false;

	switch (cmdCode)
	{
	case 200:
	case 2200:
		result = cmdGetCurrentAngle(serialNum, parameterCount, value);
		break;

	case 201:
	case 2201:
		result = cmdGetDeviceName(serialNum, parameterCount, value);
		break;

	case 202:
	case 2202:
		result = cmdGetHWVersion(serialNum, parameterCount, value);
		break;

	case 203:
	case 2203:
		result = cmdGetSWVersion(serialNum, parameterCount, value);
		break;        

	case 204:
	case 2204:
		result = cmdGetAPIVersion(serialNum, parameterCount, value);
		break;

	case 205:
	case 2205:
		result = cmdGetDeviceUUID(serialNum, parameterCount, value);
		break;   

	case 206:
	case 2206:
		result = cmdGetCurrentMotorAngle(serialNum, parameterCount, value);
		break;    		     

	case 220:
	case 2220:
		result = cmdGetCurrentXYZ(serialNum, parameterCount, value);
		break;

	case 221:
	case 2221:
		result = cmdGetCurrentPosPol(serialNum, parameterCount, value);
		break;

	case 231:
	case 2231:
		result = cmdGetPumpStatus(serialNum, parameterCount, value);
		break;

	case 232:
	case 2232:
		result = cmdGetGripperStatus(serialNum, parameterCount, value);
		break;

	case 233:
	case 2233:
		result = cmdGetTip(serialNum, parameterCount, value);
		break;  

	case 2234:
		result = cmdGetPowerStatus(serialNum, parameterCount, value);
		break;

	case 240:
	case 2240:
		result = cmdGetDigitValue(serialNum, parameterCount, value);
		break;

	case 241:
	case 2241:
		result = cmdGetAnalogValue(serialNum, parameterCount, value);
		break;

	case 2245:
		result = cmdGetCountValue(serialNum, parameterCount, value);
		break;

	default:
		replyError(serialNum, NO_SUCH_CMD);
		return;
	}

	if (result > 0)
	{
		replyError(serialNum, result);
	}
}

static bool parseCommand(char *message)
{
	double value[6];
	int index = 0;
	bool hasSerialNum = false;
	debugPrint("message=%s\r\n", message);


	int len = strlen(message);

	char *pch;
	char cmdType;

    // skip white space
	while (len > 0)
	{
		if (isspace(message[len-1]))
		{
			message[len-1] = '\0';
		}
		else
		{
			break;
		}

		len--;
	}

	if (len <= 0)
		return false;

	pch = strstr(message, "M2245 V");
	if (pch != NULL)
	{
		pch += strlen("M2245 V");
		int strLength = strlen(pch);
		char btName[20];

		if (strLength <= 0)
		{
			return false;
		}
		else if (strLength > BT_NAME_MAX_LEN)
		{
			strncpy(btName, pch, BT_NAME_MAX_LEN);
			btName[BT_NAME_MAX_LEN] = '\0';
		}
		else
		{
			strcpy(btName, pch);
		}

		cmdSetBTName(btName);
		return false;
	}

    

	if (message[0] == '#')
	{
		hasSerialNum = true;
	}

	pch = strtok(message, " ");
	while (pch != NULL && index < 6)
	{
	//debugPrint("pch=%s\r\n", pch);

		switch (index)
		{
		case 0:
			if (!hasSerialNum)
			{
				cmdType = *(pch);
			}
			value[index] = atof(pch+1);
			break;

		case 1:
			if (hasSerialNum)
			{
				cmdType = *(pch);
			}
			//debugPrint("cmdType=%d\r\n", cmdType);
			value[index] = atof(pch+1);
			break;

		default:
			value[index] = atof(pch+1);
			break;
		}


		//debugPrint("value=%f\r\n", value[index]);

		pch = strtok(NULL, " ");


		index++;

	}

	int serialNum = 0;
	int cmdCode = 0;
	int parameterCount = 0;
	int valueStartIndex = 0;

	if (hasSerialNum)
	{
		serialNum = value[0];
		cmdCode = value[1];
		parameterCount = index - 2;
		valueStartIndex = 2;
	}
	else
	{
		serialNum = 0;
		cmdCode = value[0];
		parameterCount = index - 1;        
		valueStartIndex = 1;
	}

	switch (cmdType)
	{
	case 'G':
		if (cmdCode == 203 || cmdCode == 2203 )	// stop move
		{
			HandleMoveCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
			// clear buffer
			moveCmdBuffer.clear();
		}
		else
		{
			bufferMoveCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
		}
		break;

	case 'M':
		HandleSettingCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
		break;

	case 'P':
		HandleQueryCmd(cmdCode, serialNum, parameterCount, value + valueStartIndex);
		break;

	}

	return true;

}

static void handleSerialData(char data)
{
	static unsigned char cmdCount = 0;



	switch (commState)
	{
	case IDLE:
	if (data == '#' || data == 'G' || data == 'M' || data == 'P')
	{
		commState = START;
		cmdIndex = 0;
		if (data != '#')
		{
			cmdCount = 1;   // get cmd code
		}
		else
		{
			cmdCount = 0;
		}

		cmdReceived[cmdIndex++] = data;
	}
	break;

	case START:

		if (cmdIndex >= COM_LEN_MAX)
		{

			commState = IDLE;
		}
		else if (data == '#')   // restart 
		{
			cmdIndex = 0;
			cmdCount = 0;
			cmdReceived[cmdIndex++] = data;
		} 
		else if (data == 'G' || data == 'M' || data == 'P')    
		{
			if (cmdCount >= 1)  // restart
			{
				cmdIndex = 0;
				cmdReceived[cmdIndex++] = data;
			}
			else
			{
				cmdCount++;
				cmdReceived[cmdIndex++] = data;
			}
		}
		else if (data == '\n')
		{

			cmdReceived[cmdIndex] = '\0';

			parseCommand((char*)cmdReceived);
			commState = IDLE;
		}
		else
		{

			cmdReceived[cmdIndex++] = data;
		}
		break;

	default:
		commState = IDLE;
		break;
	      
	}
}

void getSerialCmd()
{

	char data = -1;

	while (CommSerial->available())
	{
		data = CommSerial->read();

		if (data == -1)
		{
			return ;
		}
		else
		{
			handleSerialData(data);
		}
	}
}



void serialCmdInit()
{
	setSerialPort(&Serial);
}

void setSerialPortLock(bool locked)
{
	comLock = locked;
}

void setSerialPort(HardwareSerial* serial)
{
	if (serial == NULL || comLock)
	{
		return;
	}
	
	CommSerial = serial;
}

void handleSerialCmd()
{
	MoveCmd_t cmd;
	if (moveCmdBuffer.get(&cmd))
	{
		HandleMoveCmd(cmd.cmdCode, cmd.serialNum, cmd.parameterCount, cmd.parameters);
	}
}
