#include "Lights.h"
#include "Sparky.h"

Lights::Lights():
	lights(4)
{
}

void Lights::on()
{
	lights.Set(Relay::kForward);
}

void Lights::off()
{
	lights.Set(Relay::kOff);
}

int Lights::BlinkyLights(UINT32 argPtr)
{
	printf("BlinkyLights: start\n");
	Sparky* s = (Sparky*)argPtr;
	Lights* lights = s->GetLights();
	Loader* l = s->GetLoader();
	while(true)
	{
		if(l->getShooter() && l->getTop() && l->getMiddle())
		{
			lights->on();
		}
		else
		{
			if(l->getShooter())
			{
				lights->on();;
				Wait(0.2);
				lights->off();
				Wait(0.1);
			}
			if(l->getMiddle())
			{
				lights->on();
				Wait(0.2);
				lights->off();
				Wait(0.1);
			}
			if(l->getTop())
			{
				lights->on();
				Wait(0.2);
				lights->off();
				Wait(0.1);
			}
		}
		//printf("Blinking!\n");
		Wait(1.0);
	}
	printf("BlinkyLights: done\n");
	return 0;
}
