/**
  ******************************************************************************
  * @file	  uArmComm.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	  2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCOMM_H_
#define _UARMCOMM_H_

#include <Arduino.h>
#include "uArm.h"
#include "uArmBuzzer.h"
#include "uArmGrove.h"


#define COM_LEN_MAX			60


#define OUT_OF_RANGE		10
#define NO_SUCH_CMD			20
#define PARAMETER_ERROR		21
#define ADDRESS_ERROR		22
#define ERR_BUFFER_FULL		23
#define ERR_NO_POWER		24
#define ERR_FAIL    25

enum CommState
{
	IDLE,
	START,
	CMD,
	END,

	STATE_COUNT
};



#define REPORT_POS        3 
#define REPORT_BUTTON     4

void reportPos();

void reportButtonEvent(unsigned char buttonId, unsigned char event);


void serialCmdInit();

void getSerialCmd();

void handleSerialCmd();

void setSerialPortLock(bool locked);
void setSerialPort(HardwareSerial* serial);

void reportString(String string);

#endif // _UARMCOMM_H_
