#ifndef LIGHTS_H
#define LIGHTS_H

#include "WPILib.h"

class Lights
{
private:
	Relay lights;
public:
	Lights();
	static int BlinkyLights(UINT32 argPtr);
	void on();
	void off();
};

#endif
