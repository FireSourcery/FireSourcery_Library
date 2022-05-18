#include "KellyController.h"

KellyController kellyController(Serial1);

void setup(void)
{


}


void loop(void)
{
	//adread
	kellyController.writeControlThrottle(60);
}