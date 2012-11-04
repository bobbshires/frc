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

// constants
static const double BRIDGE_ARM_DOWN = 0.9;
static const double BRIDGE_ARM_UP = -0.9;
static const double BRIDGE_ARM_OFF = 0.0;
static const double AUTO_AIM_SPEED = 0.2;

class Sparky : public SimpleRobot
{
private:
	RobotDrive drive;
	Joystick stick1, stick2, stick3;
	Task targetingTask, blinkyLightsTask, autoAimTask;
	DigitalInput bridgeArmUp, bridgeArmDown;
	DriverStation *ds;
	DriverStationLCD *dsLCD;
	Victor bridgeArm;
	Relay lights;
	Targeting targeting;
	Loader loader;
	Shooter shooter;
	bool autoAimSet;
	static SEM_ID autoAimSem;
public:
	Sparky(void);
	void Disabled();
	void RobotInit();
	void Autonomous(void);
	void OperatorControl(void);
	Victor* GetBridgeArm();
	Relay* GetLights();
	Loader* GetLoader();
	Shooter* GetShooter();
	RobotDrive* GetDrive();
	int GetTension();
	static int BlinkyLights(UINT32 argPtr);	
	static int AutoAim(UINT32 argPtr);
};

#endif
