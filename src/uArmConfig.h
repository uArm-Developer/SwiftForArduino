/**
  ******************************************************************************
  * @file	uArmConfig.h
  * @author	David.Long
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONFIG_H_
#define _UARMCONFIG_H_

#include <Arduino.h>



#define SWIFT

//#define DEBUG                 // uncomment if you want to print debug info





#ifdef SWIFT
	#define HW_VER  "3.2"
	#define SW_VER  "2.3.7"
#elif defined(MKII)
	#define HW_VER  "3.1"
	#define SW_VER  "2.2.4"
#elif defined(METAL)
	#define HW_VER  "2.1"
	#define SW_VER  "2.2.4"
#else
	#error "NO machine model defined(SWIFT, METAL, MKII)"
#endif

#ifdef SWIFT
	#define DEVICE_NAME "Swift"
#elif defined(METAL)
	#define DEVICE_NAME "Metal"
#elif defined(MKII)
	#define DEVICE_NAME "MKII"
#else
	#define DEVICE_NAME "UNKNOWN"
#endif

#define TICK_INTERVAL    50    // ms

#define REPORT_SERVICE_ITEM_MAX	10



#endif // _UARMCONFIG_H_
