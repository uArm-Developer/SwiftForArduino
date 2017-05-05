/**
  ******************************************************************************
  * @file	uArmReportService.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2017-03-06
  ******************************************************************************
  */

#include "uArmReportService.h" 

ReportServiceList reportServiceList;

void reportServiceInit()
{
	reportServiceList.count = 0;
	for (int i = 0; i <REPORT_SERVICE_ITEM_MAX; i++)
	{
		reportServiceList.item[i].moduleType = GROVE_TYPE_NONE;
	}	
}

void reportServiceRun()
{
	long curTimeStamp = millis();
	for (int i = 0; i < REPORT_SERVICE_ITEM_MAX; i++)
	{
		// exist
		if (reportServiceList.item[i].moduleType != GROVE_TYPE_NONE)
		{
			if((unsigned long)(curTimeStamp - reportServiceList.item[i].timestamp) >= reportServiceList.item[i].interval)
			{
				reportServiceList.item[i].timestamp = curTimeStamp;

				if (reportServiceList.item[i].reportFuncCB != NULL)
				{
					debugPrint("reportFuncCB\r\n");
					reportServiceList.item[i].reportFuncCB();
				}
			}

		}
	}		
}

void addReportService(GroveType moduleType, long interval, void (*reportFuncCB)())
{
	if (reportFuncCB == NULL)
		return;

	if (moduleType < GROVE_COLOR_SENSOR || moduleType >= GROVE_TYPE_COUNT)
		return;

	if (interval == 0)
		return;

	debugPrint("addReportService %d\r\n", moduleType);
	int i = 0;
	for (i = 0; i < REPORT_SERVICE_ITEM_MAX; i++)
	{
		// exist
		if (reportServiceList.item[i].moduleType == moduleType)
		{
			reportServiceList.item[i].interval = interval;
			reportServiceList.item[i].timestamp = millis();
			reportServiceList.item[i].reportFuncCB = reportFuncCB;
			break;
		}
	}

	// new item
	if (i == REPORT_SERVICE_ITEM_MAX)
	{
		if (reportServiceList.count >= REPORT_SERVICE_ITEM_MAX)
		{
			// full
			return;
		}

		// find a hole
		for (int j = 0; j < REPORT_SERVICE_ITEM_MAX; j++)
		{
			if (reportServiceList.item[j].moduleType == GROVE_TYPE_NONE)
			{
				reportServiceList.item[j].moduleType = moduleType;
				reportServiceList.item[j].interval = interval;
				reportServiceList.item[j].timestamp = millis();
				reportServiceList.item[j].reportFuncCB = reportFuncCB;	
				reportServiceList.count++;	

				debugPrint("count %d\r\n", reportServiceList.count);
				return;
			}
		}


	}

}


void removeReportService(GroveType moduleType)
{
	if (moduleType < GROVE_COLOR_SENSOR || moduleType >= GROVE_TYPE_COUNT)
		return;

	for (int i = 0; i < REPORT_SERVICE_ITEM_MAX; i++)
	{
		// exist
		if (reportServiceList.item[i].moduleType == moduleType)
		{
			reportServiceList.item[i].moduleType = GROVE_TYPE_NONE;
			reportServiceList.count--;
			break;
		}
	}	
}