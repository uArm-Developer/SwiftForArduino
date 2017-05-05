/**
  ******************************************************************************
  * @file	uArmReportService.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2017-03-06
  ******************************************************************************
  */

#ifndef _UARMREPORTSERVICE_H_
#define _UARMREPORTSERVICE_H_

#include <Arduino.h>
#include "uArmDebug.h"
#include "uArmConfig.h"
#include "uArmGrove.h"

typedef struct _ReportServiceItem
{
	void (*reportFuncCB)();
	GroveType moduleType;
	long interval;
	long timestamp;
} ReportServiceItem;

typedef struct _ReportServiceList
{
	ReportServiceItem item[REPORT_SERVICE_ITEM_MAX];
	int count;
} ReportServiceList;

void reportServiceInit();
void reportServiceRun();

void addReportService(GroveType moduleType, long interval, void (*reportFuncCB)());
void removeReportService(GroveType moduleType);


#endif // _UARMREPORTSERVICE_H_
