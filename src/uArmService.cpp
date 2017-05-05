/**
  ******************************************************************************
  * @file	uArmService.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArm.h" 

#include "uArmComm.h"

uArmService service;



uArmService::uArmService()
{

	mRecordAddr = 0;

	mReportInterval = 0; 

	mButtonServiceDisable = false;

	mBTDisable = false;

	mReportStartTime = millis();

	mTickRecorderTime = millis();

#ifdef SWIFT
	mPowerState = 0;
	mTipState = 0;
#endif

}

void uArmService::setButtonService(bool on)
{
	if (on)
	{
		mButtonServiceDisable = false;
	}
	else
	{
		mButtonServiceDisable = true;
	}
}



void uArmService::setReportInterval(unsigned int interval)
{
	mReportInterval = interval;
}

#ifdef SWIFT
void menuButtonClicked()
{
	if (service.buttonServiceDisable())
	{
		reportButtonEvent(BUTTON_MENU, EVENT_CLICK);
	}
	else
	{
		service.handleButtonEvent(BUTTON_MENU, EVENT_CLICK);
	}
}

void menuButtonLongPressed()
{
	if (service.buttonServiceDisable())
	{
		reportButtonEvent(BUTTON_MENU, EVENT_LONG_PRESS);
	}
	else
	{
		service.handleButtonEvent(BUTTON_MENU, EVENT_LONG_PRESS);
	}
}
void playButtonClicked()
{
	if (service.buttonServiceDisable())
	{
		reportButtonEvent(BUTTON_PLAY, EVENT_CLICK);
	}
	else
	{
		service.handleButtonEvent(BUTTON_PLAY, EVENT_CLICK);
	}
}

void playButtonLongPressed()
{
	if (service.buttonServiceDisable())
	{
		reportButtonEvent(BUTTON_PLAY, EVENT_LONG_PRESS);
	}
	else
	{
		service.handleButtonEvent(BUTTON_PLAY, EVENT_LONG_PRESS);
	}	
}


#endif

void uArmService::init()
{
#ifdef SWIFT
	buttonMenu.setClickedCB(menuButtonClicked);
	buttonMenu.setLongPressedCB(menuButtonLongPressed);
	buttonPlay.setClickedCB(playButtonClicked);
	buttonPlay.setLongPressedCB(playButtonLongPressed);

	tickCount = 0;
#endif


}

#ifdef SWIFT
bool uArmService::buttonServiceDisable()
{
	return mButtonServiceDisable;
}

void uArmService::ledLearning()
{
	ledBlue.off();
	ledRed.off();
	ledGreen.on();
}

void uArmService::ledPlaying()
{
	ledBlue.off();
	ledRed.off();
	ledGreen.blink(500, 255);	
	// ledBlue.off();
	// ledRed.blink(500, 255);
	// ledGreen.blink(500, 50);
}

void uArmService::ledAllOff()
{
	ledRed.off();
	ledGreen.off();
	ledBlue.off();
}

void uArmService::powerDetect()
{
	unsigned char powerState = isPowerPlugIn() ? 1 : 0;

	if (powerState != mPowerState)
	{
		mPowerState = powerState;
		char buf[128];

		msprintf(buf, "@5 V%d", mPowerState);

		reportString(buf);

		if (mPowerState)
		{
			controller.powerUp();
		}

	}

}

void uArmService::tipDetect()
{
	unsigned char tipState = getTip() ? 1 : 0;

	if (tipState != mTipState)
	{
		mTipState = tipState;
		char buf[128];

		msprintf(buf, "@6 N0 V%d", mTipState);

		reportString(buf);
	}	
}


void uArmService::tick()
{
	btDetect();
	powerDetect();
	tipDetect();

	tickCount++;

	if (tickCount >= (10 * 60 * (1000/TICK_INTERVAL)))
	{
		tickCount = 0;
		controller.saveCount();
	}
}	

void uArmService::handleButtonEvent(BUTTON_ID button, unsigned char event)
{
	switch(button)
	{
	case BUTTON_MENU:
		if (event == EVENT_CLICK)
		{
			switch (mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mSysStatus = LEARNING_MODE;
				ledLearning();
				mRecordAddr = 0;//recording/playing address
				controller.detachAllServo();
				break;

			case LEARNING_MODE:
				//LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
				mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
				ledAllOff();
				pumpOff();
				gripperRelease();
				break;

			default: 
				break;
			}	
		}

		break;

	case BUTTON_PLAY:
		if (event == EVENT_CLICK)
		{
			switch(mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mRecordAddr = 0;//recording/playing address
				mSysStatus = SINGLE_PLAY_MODE;  // or play just one time
				ledPlaying();
				
				break;

			case SINGLE_PLAY_MODE:
			case LOOP_PLAY_MODE:
				pumpOff();
				gripperRelease();
				mSysStatus = NORMAL_MODE;
				ledAllOff();
				break;

			case LEARNING_MODE:
				if (buttonState.isPressed())
				{
					if (getPumpStatus())
					{
						pumpOff();
					}
					else
					{
						pumpOn();
					}    
				}
				else
				{
					if (getGripperStatus())
					{
						gripperRelease();
					}
					else
					{
						gripperCatch();
					} 					
				}
				break;
			}
		}
		else if (event == EVENT_LONG_PRESS)
		{
			switch(mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mRecordAddr = 0;
				mSysStatus = LOOP_PLAY_MODE;
				ledPlaying();
								
				break;

			case SINGLE_PLAY_MODE:
			case LOOP_PLAY_MODE:
				break;

			case LEARNING_MODE: 
				break;
			}				
		}	
		break;
	}
}
#endif

void uArmService::recorderTick()
{
    //sys led function detec every 0.05s-----------------------------------------------------------------

	switch(mSysStatus)//every 0.125s per point
	{
	case SINGLE_PLAY_MODE:
		if(play() == false)
		{
			mSysStatus = NORMAL_MODE;
			#ifdef SWIFT
			ledAllOff();
			#endif
			mRecordAddr = 0;
		}
		break;

	case LOOP_PLAY_MODE:

		if(play() == false)
		{

			mRecordAddr = 0;
		}
		break;

	case LEARNING_MODE:
	case LEARNING_MODE_STOP:
		if(record() == false)
		{
			mSysStatus = NORMAL_MODE;
			mRecordAddr = 0;
			#ifdef SWIFT
			ledAllOff();
			#endif
			controller.attachAllServo();

		}
		break;

	default: 
		break;
	}

}



void uArmService::systemRun()
{
#ifndef SWIFT
//check the button4 status------------------------------------------------------------------------
	if (mButtonServiceDisable)
	{
		if (buttonMenu.longPressed())
		{
			buttonMenu.clearEvent();
			reportButtonEvent(BUTTON_MENU, EVENT_LONG_PRESS);
		}
		else if (buttonMenu.clicked())
		{
			//debugPrint("btnD4 down");
			buttonMenu.clearEvent();
			reportButtonEvent(BUTTON_MENU, EVENT_CLICK);
		}



		//check the button7 status-------------------------------------------------------------------------
		if (buttonPlay.longPressed())
		{
			buttonPlay.clearEvent();
			reportButtonEvent(BUTTON_PLAY, EVENT_LONG_PRESS);
		}
		else if (buttonPlay.clicked())
		{
			buttonPlay.clearEvent();
			reportButtonEvent(BUTTON_PLAY, EVENT_CLICK);
		}         
	}
	else
	{
		if (buttonMenu.clicked())
		{
			//debugPrint("btnD4 down");
			buttonMenu.clearEvent();
			switch (mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mSysStatus = LEARNING_MODE;
				mRecordAddr = 0;//recording/playing address
				controller.detachAllServo();
				break;

			case LEARNING_MODE:
				//LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
				mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly

				pumpOff();

				break;

			default: 
				break;
			}
		}

        

		//check the button7 status-------------------------------------------------------------------------
		if (buttonPlay.longPressed())
		{
			buttonPlay.clearEvent();
			switch(mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mRecordAddr = 0;
				mSysStatus = LOOP_PLAY_MODE;
				break;

			case SINGLE_PLAY_MODE:
			case LOOP_PLAY_MODE:
				break;

			case LEARNING_MODE: 
				break;
			}
		}
		else if (buttonPlay.clicked())
		{
			buttonPlay.clearEvent();
                
			//debugPrint("btnD7 down");

			switch(mSysStatus)
			{
			case NORMAL_MODE:
			case NORMAL_BT_CONNECTED_MODE:
				mRecordAddr = 0;//recording/playing address
				mSysStatus = SINGLE_PLAY_MODE;  // or play just one time

				break;

			case SINGLE_PLAY_MODE:
			case LOOP_PLAY_MODE:
				pumpOff();
				mSysStatus = NORMAL_MODE;
				break;

			case LEARNING_MODE:

				if (getPumpStatus())
				{
					pumpOff();
				}
				else
				{
					pumpOn();
				}    
				break;
			}
		} 
	}
#endif

	if (mReportInterval > 0)
	{
		if(millis() - mReportStartTime >= mReportInterval)
		{
			mReportStartTime = millis();
			reportPos();
		}

	}
	
}

void uArmService::disableBT(bool disable)
{
	mBTDisable = disable;

	if (mBTDisable)
	{
		ledBlue.off();
		//#ifndef CALIBRATION
		setSerialPort(&Serial);
		//#endif
		mSysStatus = NORMAL_MODE;		
	}
}

void uArmService::btDetect()
{
#if defined( MKII )
	if (!buzzer.on() && ((mSysStatus == NORMAL_MODE) || (mSysStatus == NORMAL_BT_CONNECTED_MODE)))
	{
		pinMode(BT_DETECT_PIN, INPUT);
		digitalWrite(BT_DETECT_PIN,HIGH);

		if (digitalRead(BT_DETECT_PIN) == HIGH)//do it here
		{
			

			mSysStatus = NORMAL_BT_CONNECTED_MODE;
		}
		else
		{
			

			mSysStatus = NORMAL_MODE;
		}

		//pinMode(BT_DETECT_PIN, OUTPUT);
	}
#elif defined(SWIFT)
	if ((mSysStatus == NORMAL_MODE) || (mSysStatus == NORMAL_BT_CONNECTED_MODE))
	{
		
		digitalWrite(BT_DETECT_PIN, HIGH);

		if (digitalRead(BT_DETECT_PIN) == HIGH && !mBTDisable)
		{
			ledBlue.on();
			//#ifndef CALIBRATION
			setSerialPort(&Serial2);
			//#endif
			mSysStatus = NORMAL_BT_CONNECTED_MODE;
		}
		else
		{
			ledBlue.off();
			//#ifndef CALIBRATION
			setSerialPort(&Serial);
			//#endif
			mSysStatus = NORMAL_MODE;
		}

		//pinMode(BT_DETECT_PIN, OUTPUT);
	}
#endif

}


void uArmService::run()
{

	systemRun();

	if (millis() - mTickRecorderTime >= 50)
	{
		mTickRecorderTime= millis();
		recorderTick();
	}
}




bool uArmService::play()
{

	unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper


	recorder.read(mRecordAddr, data, 5);
	debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

	if(data[0] != 255)
	{
		//double x, y, z;
		//controller.getXYZFromAngle(x, y, z, (double)data[2], (double)data[0], (double)data[1]);
		//moveToAngle((double)data[2], (double)data[0], (double)data[1]);
		controller.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
		controller.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
		unsigned char pumpStatus = getPumpStatus() > 0 ? 1 : 0;
		if (pumpStatus != data[4])
		{
			if (data[4])
			{
				pumpOn();
			}
			else
			{
				pumpOff();
			}   
		}
	}
	else
	{

		pumpOff();

		return false;
	}

	mRecordAddr += 5;

	return true;
}

bool uArmService::record()
{
	debugPrint("mRecordAddr = %d", mRecordAddr);

	if(mRecordAddr <= 65530)
	{
		unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
		if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP))
		{
			double rot, left, right;
			//controller.updateAllServoAngle();
			controller.readServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
			data[2] = (unsigned char)rot;
			data[3] = (unsigned char)controller.readServoAngle(SERVO_HAND_ROT_NUM);
			data[4] = getPumpStatus() > 0 ? 1 : 0;

			debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
		}
		else
		{
			data[0] = 255;//255 is the ending flag
			recorder.write(mRecordAddr, data, 5);

			return false;
		}

		recorder.write(mRecordAddr, data, 5);
		mRecordAddr += 5;

		return true;
	}
	else
	{
		return false;
	}

}