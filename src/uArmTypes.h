/**
  ******************************************************************************
  * @file	uArmTypes.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-12-12
  ******************************************************************************
  */

#ifndef _UARMTYPES_H_
#define _UARMTYPES_H_

// gripper or pump status
#define STOP            		0
#define WORKING          		1
#define GRABBING        		2

//return values
#define OK								0

#define IN_RANGE             			1
#define OUT_OF_RANGE_NO_SOLUTION 		2
#define OUT_OF_RANGE         			3
#define NO_NEED_TO_MOVE					4

#define ERR_SERVO_INDEX_EXCEED_LIMIT	5
#define ERR_ANGLE_OUT_OF_RANGE			6
#define ERR_NO_POWER_CONNECTED			7

// e2prom device
#define EEPROM_ON_CHIP			0
#define EEPROM_EXTERN_USER		1
#define EEPROM_EXTERN_SYSTEM	2

// e2prom data type
#define DATA_TYPE_BYTE          1
#define DATA_TYPE_INTEGER       2
#define DATA_TYPE_FLOAT         4

// servo define
#define SERVO_ROT_NUM           0
#define SERVO_LEFT_NUM          1
#define SERVO_RIGHT_NUM         2
#define SERVO_HAND_ROT_NUM      3
#define SERVO_COUNT				4

#define BT_NAME_MAX_LEN		11

#define EERPOM_COUNT_ADDR	80

#define BT_UUID	0xFFE1


#define REPORT_TYPE_POS		3
#define REPORT_TYPE_GROVE	10
#endif // _UARMTYPES_H_
