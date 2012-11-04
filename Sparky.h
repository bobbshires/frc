#ifndef SPARKY_H
#define SPARKY_H

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "NiVision.h"
#include "math.h"
#include "Targeting.h"
#include "Loader.h"
#include "Shooter.h"
#include "BridgeArm.h"

class Sparky : public SimpleRobot
{
private:
	RobotDrive drive;
	Joystick stick1, stick2, stick3;
	Task targetingTask, blinkyLightsTask, autoAimTask;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Relay lights;
	Targeting targeting;
	Loader loader;
	Shooter shooter;
	BridgeArm bridgeArm;
public:
	Sparky(void);
	void Disabled();
	void RobotInit();
	void Autonomous(void);
	void OperatorControl(void);
	Relay* GetLights();
	Loader* GetLoader();
	Shooter* GetShooter();
	RobotDrive* GetDrive();
	Targeting* GetTargeting();
	int GetTension();
	static int BlinkyLights(UINT32 argPtr);	
};

#endif
