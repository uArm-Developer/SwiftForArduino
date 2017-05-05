/**
  ******************************************************************************
  * @file	uArmButton.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-17
  ******************************************************************************
  */

#include "uArmButton.h" 



uArmButton::uArmButton()
{
	mPin = 0xff;
	mState = INIT;
	mEvent = EVENT_NONE;

#ifdef SWIFT
	pButtonEventCB = NULL;
	pButtonLongPressedCB = NULL;
#endif
}

#ifdef SWIFT
void uArmButton::setClickedCB(ButtonEventCB_t clickedCB)
{
	pButtonEventCB = clickedCB;
}

void uArmButton::setLongPressedCB(ButtonLongPressedCB_t longPressedCB)
{
	pButtonLongPressedCB = longPressedCB;
}
#endif

void uArmButton::setPin(unsigned char pin)
{
	mPin = pin;
	mState = IDLE;
}

bool uArmButton::clicked()
{
	return (mEvent == EVENT_CLICK);
}

bool uArmButton::longPressed()
{
	return (mEvent == EVENT_LONG_PRESS);
}

bool uArmButton::isPressed()
{
	if (!digitalRead(mPin))
		return true;

	return false;
}

void uArmButton::clearEvent()
{
	mEvent = EVENT_NONE;
}

void uArmButton::tick()
{
	switch (mState)
	{
	case IDLE:
		if (isPressed())
		{
			mTicks = 0;
			mState = HALF_PRESSED;
		}
		break;

	case HALF_PRESSED:
		if (isPressed())
		{
			buzzer.buzz(4000, 100);
			mState = PRESSED;
		}
		else
		{
			mState = IDLE;
		}	
		break;

	case PRESSED:
		if (isPressed())
		{

			mTicks++;
		}
		else
		{
			mState = RELEASE;
		}		
		break;

	case RELEASE:

		if (mTicks >= (1000/TICK_INTERVAL))
		{
			mEvent = EVENT_LONG_PRESS;
			#ifdef SWIFT
				if (pButtonLongPressedCB != NULL)
				{
					pButtonLongPressedCB();
				}
			#endif
		}
		else
		{
			mEvent = EVENT_CLICK;
			#ifdef SWIFT
				if (pButtonEventCB != NULL)
				{
					pButtonEventCB();
				}
			#endif			
		}

		mTicks = 0;
		mState = IDLE;
		break;

	default:
		break;

	}
}