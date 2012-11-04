#ifndef LOADER_H
#define LOADER_H

#include "WPILib.h"

static const double INTAKE_LOAD = 1.0;
static const double INTAKE_UNLOAD = -1.0;
static const double INTAKE_OFF = 0.0;
static const double ARM_ZERO_THRESH = 75;

class Loader
{
private:
	DigitalInput top, middle, shooter;
	Victor floorPickup, shooterLoader;
	bool intakeOff;
public:
	Loader();
	UINT32 getTop();
	UINT32 getMiddle();
	UINT32 getShooter();
	void load(Timer& armTimer, int tension);
	void load();
	void unload();
	void stop();
	bool isIntakeOff();
	void setIntakeOff(bool b);
};

#endif
