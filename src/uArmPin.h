/**
  ******************************************************************************
  * @file	uArmPin.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-17
  ******************************************************************************
  */

#ifndef _UARMPIN_H_
#define _UARMPIN_H_

#include "uArmConfig.h"

#define EXTERNAL_EEPROM_SYS_ADDRESS 0xA2
#define EXTERNAL_EEPROM_USER_ADDRESS  0xA0

#ifdef SWIFT
    #define BUZZER          		5    // HIGH = ON
	#define BT_DETECT_PIN			39

	#define LIMIT_SW                28    // LIMIT Switch Button

	#define BTN_D4                  37    // LOW = Pressed
	#define BTN_D7                  36    // LOW = Pressed

	#define BTN_STATE				29

	#define LED_R					13
	#define LED_G					12
	#define LED_B					11

	#define PUMP_EN                 27   
	#define VALVE_EN                26

	#define GRIPPER                 18    
	#define GRIPPER_FEEDBACK        19

	#define PUMP_FEEDBACK			A4

	#define POWER_DETECT			A7


	#define SERVO_ROT_PIN           23
	#define SERVO_LEFT_PIN          25
	#define SERVO_RIGHT_PIN         24
	#define SERVO_HAND_ROT_PIN      22

	#define SERVO_ROT_ANALOG_PIN 		2
	#define SERVO_LEFT_ANALOG_PIN 		0
	#define SERVO_RIGHT_ANALOG_PIN 		1
	#define SERVO_HAND_ROT_ANALOG_PIN 	3	

#elif defined(MKII)

    #define BUZZER          		3    // HIGH = ON
	#define BT_DETECT_PIN			3

	#define LIMIT_SW                2    // LIMIT Switch Button

	#define BTN_D4                  7    // LOW = Pressed
	#define BTN_D7                  4    // LOW = Pressed

	#define SYS_LED					6

	#define PUMP_EN                 5    
	#define GRIPPER                 5    
	#define GRIPPER_FEEDBACK        A7
	#define PUMP_FEEDBACK			A7

	#define POWER_DETECT			A6


	#define SERVO_ROT_PIN           11
	#define SERVO_LEFT_PIN          13
	#define SERVO_RIGHT_PIN         12
	#define SERVO_HAND_ROT_PIN      10

	#define SERVO_ROT_ANALOG_PIN 		2
	#define SERVO_LEFT_ANALOG_PIN 		0
	#define SERVO_RIGHT_ANALOG_PIN 		1
	#define SERVO_HAND_ROT_ANALOG_PIN 	3

#elif defined(METAL)

	#define BUZZER                  3    // HIGH = ON
	#define LIMIT_SW                2    // LIMIT Switch Button

	#define BTN_D4                  4    // LOW = Pressed
	#define BTN_D7                  7    // LOW = Pressed

	#define PUMP_EN                 6    // HIGH = Valve OPEN
	#define VALVE_EN                5    // HIGH = Pump ON
	#define GRIPPER                 9    // LOW = Catch
	#define GRIPPER_FEEDBACK        A6

	#define SERVO_ROT_PIN           11
	#define SERVO_LEFT_PIN          13
	#define SERVO_RIGHT_PIN         12
	#define SERVO_HAND_ROT_PIN      10

	#define SERVO_ROT_ANALOG_PIN 		2
	#define SERVO_LEFT_ANALOG_PIN 		0
	#define SERVO_RIGHT_ANALOG_PIN 		1
	#define SERVO_HAND_ROT_ANALOG_PIN 	3

#endif	// MKII

#endif // _UARMPIN_H_
