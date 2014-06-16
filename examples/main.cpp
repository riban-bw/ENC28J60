/**     Source file for Arduino projects
        Based on main.c from Arduino core library
        Must be linked to arduino core library
*/

#include "Arduino.h"
void setup(); //declare setup
void loop(); //declare loop

int main(void)
{
	init(); //initiate core library

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup(); //run sketch setup code

    //main program loop
	for (;;)
    {
		loop();
		if (serialEventRun)
            serialEventRun();
	}

	return 0; //never get here but should be seen to return something!
}
