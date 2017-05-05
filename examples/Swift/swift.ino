#include "uArm.h"

#define USE_SERIAL_CMD	1	// 1: use serial for control	0: just use arduino to control(release ROM and RAM space)

unsigned long tickStartTime = millis(); // get timestamp;
static void Init();

void setup()
{
	Serial.begin(115200);
#ifdef SWIFT	
	Serial2.begin(115200);		// Bluetooth
#endif 
	Init(); // Don't remove

	debugPrint("debug start"); // uncomment DEBUG in uArmConfig.h to use debug function
	
	// TODO
	moveTo(150, 0, 150);
	Serial.println("@1");	// report ready
	

}

void loop()
{
	run(); // Don't remove

	// TODO

}

// time out every TICK_INTERVAL(50 ms default)
void tickTimeOut()
{
	
}

////////////////////////////////////////////////////////////
// DO NOT EDIT
static void Init()
{
	uArmInit();	// Don't remove
	service.init();

	#if USE_SERIAL_CMD == 1
	serialCmdInit();
	#endif
}

void run()
{
	#if USE_SERIAL_CMD == 1
	handleSerialCmd();
	#endif

	manage_inactivity(); // Don't remove
}

void tickTaskRun()
{
	tickTimeOut();

	buttonPlay.tick();
	buttonMenu.tick();
#ifdef MKII
	ledRed.tick();
	service.btDetect();
#elif defined(SWIFT)
	ledRed.tick();
	ledGreen.tick();
	ledBlue.tick();
	service.tick();
#endif    
}

void manage_inactivity(void)
{
#if USE_SERIAL_CMD == 1
	getSerialCmd();	// for serial communication
#endif
	service.run();	// for led, button, bt etc.
	reportServiceRun();	
	
	// because there is no other hardware timer available in UNO, so use a soft timer
	// it's necessary for button,led, bt
	// so Don't remove it if you need them
	if(millis() - tickStartTime >= TICK_INTERVAL)
	{
		tickStartTime = millis();
		tickTaskRun();
	}   
}
