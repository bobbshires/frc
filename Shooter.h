#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"

static const double MOTOR_OFF = 0.0;
static const double TENSION_BRAKE = -0.06;
static const double ARM_SPEED_COARSE = 0.5;
static const double ARM_SPEED_COARSE_LOAD = -0.5;
static const double ARM_SPEED_COARSE_UNLOAD = 0.5;
static const double ARM_SPEED_FINE_LOAD = -0.3;
static const double ARM_SPEED_FINE_UNLOAD = 0.2;
static const double ARM_SPEED_FULL_LOAD = -1.0;
static const double ARM_SPEED_FULL_UNLOAD = 1.0;

class Shooter
{
private:
	Encoder tension;
	DigitalInput shooter, trigger;
	Relay release;
	Jaguar arm;
	static bool armSet;
	static int encPos;
	static double armSpeed;
	static bool releaseSet;
	static SEM_ID armSem;
	static SEM_ID releaseSem;
public:
	Shooter();
	void setArmSet(bool b);
	bool isArmSet();
	static void ReleaseNotifier(void* p);
	int getEncPos();
	void setEncPos(int i);
	double getArmSpeed();
	void setArmSpeed(double d);
	bool isReleaseSet();
	void setReleaseSet(bool b);
	int getTension();
	void zero();
	void load(double s);
	static void ArmToPositionNotifier(void* p);
	void setRelease(Relay::Value v);
	int getTrigger();
	
};

#endif
